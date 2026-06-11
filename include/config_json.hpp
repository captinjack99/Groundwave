/**
 * @file config_json.hpp
 * @brief JSON serialization/deserialization for AppState config
 *
 * Saves/loads: presets (8 slots), alarm thresholds, active preset, TX config.
 * Pure C++17 — no Qt dependency. Hand-rolled JSON for zero external deps.
 * Lives in gw_core so config can be tested headless.
 */
#pragma once

#include "app_state.hpp"
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>

namespace gw {

// =========================================================================
// JSON Writer (minimal, generates compact JSON)
// =========================================================================

class JsonWriter {
public:
    std::string result() const { return ss_.str(); }

    void beginObject()     { comma(); ss_ << '{'; needs_comma_ = false; }
    void endObject()       { ss_ << '}'; needs_comma_ = true; }
    void beginArray()      { comma(); ss_ << '['; needs_comma_ = false; }
    void endArray()        { ss_ << ']'; needs_comma_ = true; }

    void key(const char* k) {
        comma(); ss_ << '"' << k << "\":";
        needs_comma_ = false;
    }

    void valueStr(const char* v) {
        comma();
        ss_ << '"';
        for (const char* p = v; *p; ++p) {
            switch (*p) {
                case '"':  ss_ << "\\\""; break;
                case '\\': ss_ << "\\\\"; break;
                case '\n': ss_ << "\\n";  break;
                default:   ss_ << *p;
            }
        }
        ss_ << '"';
    }

    void valueInt(int64_t v)    { comma(); ss_ << v; }
    void valueBool(bool v)      { comma(); ss_ << (v ? "true" : "false"); }
    void valueFloat(double v)   {
        comma();
        // JSON has no nan/inf literal; "%.8g" would emit "nan"/"inf" which
        // our reader parses as null and silently drops the field. Emit 0
        // for any non-finite value so the document stays valid and the
        // field round-trips to a sane default. (#59)
        if (!std::isfinite(v)) v = 0.0;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.8g", v);
        ss_ << buf;
    }

private:
    void comma() { if (needs_comma_) ss_ << ','; needs_comma_ = true; }
    std::ostringstream ss_;
    bool needs_comma_ = false;
};

// =========================================================================
// JSON Reader (minimal recursive-descent parser)
// =========================================================================

struct JsonValue;
using JsonObject = std::vector<std::pair<std::string, JsonValue>>;
using JsonArray  = std::vector<JsonValue>;

struct JsonValue {
    enum Type { Null, Bool, Number, String, Object, Array } type = Null;
    bool        b = false;
    double      n = 0.0;
    std::string s;
    JsonObject  obj;
    JsonArray   arr;

    bool has(const char* key) const {
        for (auto& kv : obj) if (kv.first == key) return true;
        return false;
    }
    const JsonValue& at(const char* key) const {
        for (auto& kv : obj) if (kv.first == key) return kv.second;
        static JsonValue null_val;
        return null_val;
    }
    int    asInt()    const { return static_cast<int>(n); }
    float  asFloat()  const { return static_cast<float>(n); }
    bool   asBool()   const { return b; }
    const std::string& asStr() const { return s; }
};

class JsonReader {
public:
    static JsonValue parse(const std::string& json) {
        JsonReader r(json);
        return r.parseValue();
    }

private:
    explicit JsonReader(const std::string& json) : src_(json), pos_(0) {}

    void skipWS() {
        while (pos_ < src_.size() && std::isspace(static_cast<unsigned char>(src_[pos_])))
            ++pos_;
    }

    char peek() { skipWS(); return pos_ < src_.size() ? src_[pos_] : '\0'; }
    char next() { skipWS(); return pos_ < src_.size() ? src_[pos_++] : '\0'; }

    JsonValue parseValue() {
        char c = peek();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == '"') return parseString();
        if (c == 't' || c == 'f') return parseBool();
        if (c == 'n') return parseNull();
        return parseNumber();
    }

    JsonValue parseObject() {
        JsonValue v; v.type = JsonValue::Object;
        next(); // '{'
        if (peek() == '}') { next(); return v; }
        while (true) {
            // Truncated input: once we run out of bytes, every helper returns
            // '\0'/empty WITHOUT advancing pos_, so the original `while(true)`
            // spun forever appending empty entries (a malformed-input hang /
            // OOM DoS — surfaced by the #25 fuzz suite). Bail on EOF.
            if (pos_ >= src_.size()) break;
            auto key = parseString();
            next(); // ':'
            auto val = parseValue();
            v.obj.push_back({key.s, std::move(val)});
            if (peek() == '}') { next(); return v; }
            if (pos_ >= src_.size()) break;   // no closing '}' before EOF
            next(); // ','
        }
        return v;
    }

    JsonValue parseArray() {
        JsonValue v; v.type = JsonValue::Array;
        next(); // '['
        if (peek() == ']') { next(); return v; }
        while (true) {
            if (pos_ >= src_.size()) break;   // truncated input: stop at EOF
            v.arr.push_back(parseValue());
            if (peek() == ']') { next(); return v; }
            if (pos_ >= src_.size()) break;   // no closing ']' before EOF
            next(); // ','
        }
        return v;
    }

    JsonValue parseString() {
        JsonValue v; v.type = JsonValue::String;
        next(); // opening '"'
        while (pos_ < src_.size() && src_[pos_] != '"') {
            if (src_[pos_] == '\\') {
                ++pos_;
                if (pos_ < src_.size()) {
                    switch (src_[pos_]) {
                        case '"':  v.s += '"';  break;
                        case '\\': v.s += '\\'; break;
                        case 'n':  v.s += '\n'; break;
                        case 't':  v.s += '\t'; break;
                        default:   v.s += src_[pos_];
                    }
                }
            } else {
                v.s += src_[pos_];
            }
            ++pos_;
        }
        if (pos_ < src_.size()) ++pos_; // closing '"'
        return v;
    }

    JsonValue parseNumber() {
        JsonValue v; v.type = JsonValue::Number;
        skipWS();
        size_t start = pos_;
        if (pos_ < src_.size() && src_[pos_] == '-') ++pos_;
        while (pos_ < src_.size() && (std::isdigit(static_cast<unsigned char>(src_[pos_]))
                || src_[pos_] == '.' || src_[pos_] == 'e' || src_[pos_] == 'E'
                || src_[pos_] == '+' || src_[pos_] == '-'))
            ++pos_;
        v.n = std::strtod(src_.c_str() + start, nullptr);
        return v;
    }

    JsonValue parseBool() {
        JsonValue v; v.type = JsonValue::Bool;
        if (src_.substr(pos_, 4) == "true")  { v.b = true;  pos_ += 4; }
        else                                  { v.b = false; pos_ += 5; }
        return v;
    }

    JsonValue parseNull() {
        JsonValue v; v.type = JsonValue::Null;
        pos_ += 4; // "null"
        return v;
    }

    const std::string& src_;
    size_t pos_;
};

// =========================================================================
// Serialization helpers
// =========================================================================

inline const char* modulationToStr(Modulation m) {
    switch (m) {
        case Modulation::BPSK:    return "BPSK";
        case Modulation::QPSK:    return "QPSK";
        case Modulation::QAM16:   return "QAM16";
        case Modulation::QAM64:   return "QAM64";
        case Modulation::QAM256:  return "QAM256";
        case Modulation::QAM1024: return "QAM1024";
        case Modulation::QAM4096: return "QAM4096";
        default: return "QPSK";
    }
}

inline Modulation strToModulation(const std::string& s) {
    if (s == "BPSK")    return Modulation::BPSK;
    if (s == "QPSK")    return Modulation::QPSK;
    if (s == "QAM16")   return Modulation::QAM16;
    if (s == "QAM64")   return Modulation::QAM64;
    if (s == "QAM256")  return Modulation::QAM256;
    if (s == "QAM1024") return Modulation::QAM1024;
    if (s == "QAM4096") return Modulation::QAM4096;
    return Modulation::QPSK;
}

inline const char* fecRateToStr(FECRate r) {
    switch (r) {
        case FECRate::Rate_1_4:  return "1/4";
        case FECRate::Rate_1_3:  return "1/3";
        case FECRate::Rate_2_5:  return "2/5";
        case FECRate::Rate_1_2:  return "1/2";
        case FECRate::Rate_3_5:  return "3/5";
        case FECRate::Rate_2_3:  return "2/3";
        case FECRate::Rate_3_4:  return "3/4";
        case FECRate::Rate_4_5:  return "4/5";
        case FECRate::Rate_5_6:  return "5/6";
        case FECRate::Rate_8_9:  return "8/9";
        case FECRate::Rate_9_10: return "9/10";
        case FECRate::None:      return "none";
        default: return "1/2";
    }
}

inline FECRate strToFECRate(const std::string& s) {
    if (s == "1/4")  return FECRate::Rate_1_4;
    if (s == "1/3")  return FECRate::Rate_1_3;
    if (s == "2/5")  return FECRate::Rate_2_5;
    if (s == "1/2")  return FECRate::Rate_1_2;
    if (s == "3/5")  return FECRate::Rate_3_5;
    if (s == "2/3")  return FECRate::Rate_2_3;
    if (s == "3/4")  return FECRate::Rate_3_4;
    if (s == "4/5")  return FECRate::Rate_4_5;
    if (s == "5/6")  return FECRate::Rate_5_6;
    if (s == "8/9")  return FECRate::Rate_8_9;
    if (s == "9/10") return FECRate::Rate_9_10;
    if (s == "none") return FECRate::None;
    return FECRate::Rate_1_2;
}

inline const char* cpToStr(CyclicPrefix cp) {
    switch (cp) {
        case CyclicPrefix::CP_1_4:  return "1/4";
        case CyclicPrefix::CP_1_8:  return "1/8";
        case CyclicPrefix::CP_1_16: return "1/16";
        case CyclicPrefix::CP_1_32: return "1/32";
        default: return "1/8";
    }
}

inline CyclicPrefix strToCP(const std::string& s) {
    if (s == "1/4")  return CyclicPrefix::CP_1_4;
    if (s == "1/8")  return CyclicPrefix::CP_1_8;
    if (s == "1/16") return CyclicPrefix::CP_1_16;
    if (s == "1/32") return CyclicPrefix::CP_1_32;
    return CyclicPrefix::CP_1_8;
}

inline const char* streamSourceToStr(StreamAudioSource s) {
    switch (s) {
        case StreamAudioSource::TestTone:   return "tone";
        case StreamAudioSource::Silence:    return "silence";
        case StreamAudioSource::Microphone: return "mic";
        case StreamAudioSource::File:       return "file";
    }
    return "tone";
}

inline StreamAudioSource strToStreamSource(const std::string& s) {
    if (s == "silence") return StreamAudioSource::Silence;
    if (s == "mic")     return StreamAudioSource::Microphone;
    if (s == "file")    return StreamAudioSource::File;
    return StreamAudioSource::TestTone;
}

// =========================================================================
// Serialize AppState config → JSON string
// =========================================================================

inline std::string serializeConfig(const AppState& state) {
    JsonWriter w;
    w.beginObject();

    // ---- Version ----
    w.key("version"); w.valueInt(2);

    // ---- Active config ----
    w.key("active_preset"); w.valueInt(state.active_preset_slot);
    w.key("tx_enabled");    w.valueBool(state.tx_enabled);
    w.key("tx_gain_db");    w.valueFloat(state.tx_gain_db);

    // ---- Current OFDM params ----
    w.key("ofdm"); w.beginObject();
    w.key("fft_size");      w.valueInt(state.ofdm.fft_size);
    w.key("modulation");    w.valueStr(modulationToStr(state.ofdm.modulation));
    w.key("cyclic_prefix"); w.valueStr(cpToStr(state.ofdm.cyclic_prefix));
    w.key("pilot_spacing"); w.valueInt(state.ofdm.pilot_spacing);
    w.key("pilot_boost");   w.valueFloat(state.ofdm.pilot_boost_db);
    w.key("dc_null");       w.valueBool(state.ofdm.dc_null);
    w.key("sample_rate");   w.valueInt(state.ofdm.sample_rate);
    w.key("guard_left");    w.valueInt(state.ofdm.guard_left);
    w.key("guard_right");   w.valueInt(state.ofdm.guard_right);
    w.key("cp_window_pct"); w.valueFloat(state.ofdm.cp_window_taper_pct);
    w.endObject();

    // ---- Frame params ----
    w.key("frame"); w.beginObject();
    w.key("fec_rate");           w.valueStr(fecRateToStr(state.frame.fec_rate));
    w.key("preamble_interval");  w.valueInt(state.frame.preamble_interval);
    w.endObject();

    // ---- Modem config ----
    w.key("modem"); w.beginObject();
    w.key("sample_rate");  w.valueInt(state.modem.sample_rate);
    w.key("center_freq");  w.valueFloat(state.modem.center_freq);
    w.key("signal_bw");    w.valueFloat(state.modem.signal_bw);
    w.key("lpf_taps");     w.valueInt(static_cast<int64_t>(state.modem.lpf_taps));
    w.key("tx_lpf_taps");  w.valueInt(static_cast<int64_t>(state.modem.tx_lpf_taps));
    w.key("enable_rs_outer"); w.valueBool(state.modem.enable_rs_outer);
    // User-settable DSP toggles that previously reset to defaults on load.
    w.key("enable_afc");        w.valueBool(state.modem.enable_afc);
    w.key("enable_rx_squelch"); w.valueBool(state.modem.enable_rx_squelch);
    w.key("enable_dc_blocker"); w.valueBool(state.modem.enable_dc_blocker);
    w.key("rx_gain_db");        w.valueFloat(state.modem.rx_gain_db);
    // AGC + squelch numerics — all Tuning-panel-editable; without these a
    // Save→Load round-trip silently reverted the panel's work.
    w.key("agc_target_rms");    w.valueFloat(state.modem.agc.target_rms);
    w.key("agc_attack_ms");     w.valueFloat(state.modem.agc.attack_ms);
    w.key("agc_release_ms");    w.valueFloat(state.modem.agc.release_ms);
    w.key("agc_max_gain");      w.valueFloat(state.modem.agc.max_gain);
    w.key("squelch_open_db");   w.valueFloat(state.modem.squelch.open_threshold_db);
    w.key("squelch_close_db");  w.valueFloat(state.modem.squelch.close_threshold_db);
    w.endObject();

    // ---- Hierarchical modulation config ----
    w.key("hier"); w.beginObject();
    w.key("enabled"); w.valueBool(state.hier.enabled);
    w.key("mode");    w.valueInt(static_cast<int>(state.hier.mode));
    w.key("alpha");   w.valueFloat(state.hier.alpha);
    w.key("hp_bits"); w.valueInt(state.hier.hp_bits);
    w.key("lp_bits"); w.valueInt(state.hier.lp_bits);
    w.endObject();

    // ---- Link-budget panel inputs ----
    w.key("link_budget"); w.beginObject();
    w.key("tx_power_w");  w.valueFloat(state.link_budget.tx_power_w);
    w.key("tx_gain_db");  w.valueFloat(state.link_budget.tx_gain_db);
    w.key("rx_gain_db");  w.valueFloat(state.link_budget.rx_gain_db);
    w.key("cable_loss_db"); w.valueFloat(state.link_budget.cable_loss_db);
    w.key("freq_mhz");    w.valueFloat(state.link_budget.freq_mhz);
    w.key("tx_height_m"); w.valueFloat(state.link_budget.tx_height_m);
    w.key("rx_height_m"); w.valueFloat(state.link_budget.rx_height_m);
    w.key("terrain_idx"); w.valueInt(state.link_budget.terrain_idx);
    w.key("nf_db");       w.valueFloat(state.link_budget.nf_db);
    w.key("margin_db");   w.valueFloat(state.link_budget.margin_db);
    w.endObject();

    // ---- Alarm thresholds ----
    w.key("alarms"); w.beginObject();
    w.key("snr_low_db");       w.valueFloat(state.alarm_thresh.snr_low_db);
    w.key("ber_high");         w.valueFloat(state.alarm_thresh.ber_high);
    w.key("evm_high_pct");     w.valueFloat(state.alarm_thresh.evm_high_pct);
    w.key("level_low_db");     w.valueFloat(state.alarm_thresh.level_low_db);
    w.key("level_high_db");    w.valueFloat(state.alarm_thresh.level_high_db);
    w.key("alarm_sync_loss");  w.valueBool(state.alarm_thresh.alarm_sync_loss);
    w.key("alarm_audio_clip"); w.valueBool(state.alarm_thresh.alarm_audio_clip);
    w.endObject();

    // ---- Stream configs ----
    w.key("streams"); w.beginArray();
    for (size_t i = 0; i < MAX_STREAMS; ++i) {
        const auto& sc = state.stream_configs[i];
        w.beginObject();
        w.key("enabled");        w.valueBool(sc.enabled);
        w.key("name");           w.valueStr(sc.name);
        w.key("bitrate_bps");    w.valueInt(sc.bitrate_bps);
        w.key("weight");         w.valueFloat(sc.weight);
        w.key("channels");       w.valueInt(sc.channels);
        w.key("frame_ms");       w.valueInt(sc.frame_ms);
        w.key("sample_rate");    w.valueInt(sc.sample_rate);
        w.key("source");         w.valueStr(streamSourceToStr(sc.source));
        w.key("tone_freq_hz");   w.valueFloat(sc.tone_freq_hz);
        w.key("tone_amplitude"); w.valueFloat(sc.tone_amplitude);
        w.key("file_path");      w.valueStr(sc.file_path);
        w.key("input_device");   w.valueInt(sc.input_device);
        w.key("opus_dred_frames"); w.valueInt(sc.opus_dred_frames);
        // Opus application (encoder mode) + Mid/Side split — both change the
        // built encoder, so dropping them silently reverted VoIP/LowDelay
        // streams to Audio and any custom M/S split to the default on load.
        w.key("app");            w.valueInt(static_cast<int>(sc.app));
        w.key("mid_side_split"); w.valueFloat(sc.mid_side_split);
        w.endObject();
    }
    w.endArray();

    // ---- Presets ----
    w.key("presets"); w.beginArray();
    for (size_t i = 0; i < NUM_PRESETS; ++i) {
        auto& p = state.presets[i];
        w.beginObject();
        w.key("name");      w.valueStr(p.name);
        w.key("valid");     w.valueBool(p.valid);
        if (p.valid) {
            w.key("fft_size");      w.valueInt(p.ofdm.fft_size);
            w.key("modulation");    w.valueStr(modulationToStr(p.ofdm.modulation));
            w.key("cyclic_prefix"); w.valueStr(cpToStr(p.ofdm.cyclic_prefix));
            w.key("fec_rate");      w.valueStr(fecRateToStr(p.frame.fec_rate));
            w.key("sample_rate");   w.valueInt(p.ofdm.sample_rate);
            w.key("center_freq");   w.valueFloat(p.modem.center_freq);
            w.key("pilot_spacing"); w.valueInt(p.ofdm.pilot_spacing);
            w.key("pilot_boost");   w.valueFloat(p.ofdm.pilot_boost_db);
            w.key("dc_null");       w.valueBool(p.ofdm.dc_null);
            w.key("guard_left");    w.valueInt(p.ofdm.guard_left);
            w.key("guard_right");   w.valueInt(p.ofdm.guard_right);
            w.key("cp_window_pct"); w.valueFloat(p.ofdm.cp_window_taper_pct);
            w.key("preamble_interval"); w.valueInt(p.frame.preamble_interval);
            w.key("signal_bw");     w.valueFloat(p.modem.signal_bw);
            w.key("lpf_taps");      w.valueInt(static_cast<int64_t>(p.modem.lpf_taps));
            w.key("tx_lpf_taps");   w.valueInt(static_cast<int64_t>(p.modem.tx_lpf_taps));
            w.key("enable_rs_outer"); w.valueBool(p.modem.enable_rs_outer);
        }
        w.endObject();
    }
    w.endArray();

    w.endObject();
    return w.result();
}

// =========================================================================
// Deserialize JSON string → AppState config
// =========================================================================

inline bool deserializeConfig(const std::string& json, AppState& state) {
    try {
        auto root = JsonReader::parse(json);
        if (root.type != JsonValue::Object) return false;

        // Version check
        if (root.has("version") && root.at("version").asInt() != 2) return false;

        // Active config
        if (root.has("active_preset"))
            state.active_preset_slot = root.at("active_preset").asInt();
        if (root.has("tx_enabled"))
            state.tx_enabled = root.at("tx_enabled").asBool();
        if (root.has("tx_gain_db"))
            state.tx_gain_db = root.at("tx_gain_db").asFloat();

        // OFDM params
        if (root.has("ofdm")) {
            auto& o = root.at("ofdm");
            if (o.has("fft_size"))      state.ofdm.fft_size = static_cast<uint16_t>(o.at("fft_size").asInt());
            if (o.has("modulation"))    state.ofdm.modulation = strToModulation(o.at("modulation").asStr());
            if (o.has("cyclic_prefix")) state.ofdm.cyclic_prefix = strToCP(o.at("cyclic_prefix").asStr());
            if (o.has("pilot_spacing")) state.ofdm.pilot_spacing = static_cast<uint8_t>(o.at("pilot_spacing").asInt());
            if (o.has("pilot_boost"))   state.ofdm.pilot_boost_db = o.at("pilot_boost").asFloat();
            if (o.has("dc_null"))       state.ofdm.dc_null = o.at("dc_null").asBool();
            if (o.has("sample_rate"))   state.ofdm.sample_rate = static_cast<uint32_t>(o.at("sample_rate").asInt());
            if (o.has("guard_left"))    state.ofdm.guard_left = static_cast<uint16_t>(o.at("guard_left").asInt());
            if (o.has("guard_right"))   state.ofdm.guard_right = static_cast<uint16_t>(o.at("guard_right").asInt());
            if (o.has("cp_window_pct")) state.ofdm.cp_window_taper_pct = o.at("cp_window_pct").asFloat();
        }

        // Frame params
        if (root.has("frame")) {
            auto& f = root.at("frame");
            if (f.has("fec_rate"))          state.frame.fec_rate = strToFECRate(f.at("fec_rate").asStr());
            if (f.has("preamble_interval")) state.frame.preamble_interval = static_cast<uint32_t>(f.at("preamble_interval").asInt());
        }

        // Modem config
        if (root.has("modem")) {
            auto& m = root.at("modem");
            if (m.has("sample_rate")) state.modem.sample_rate = static_cast<uint32_t>(m.at("sample_rate").asInt());
            if (m.has("center_freq")) state.modem.center_freq = m.at("center_freq").asFloat();
            if (m.has("signal_bw"))   state.modem.signal_bw = m.at("signal_bw").asFloat();
            if (m.has("lpf_taps"))    state.modem.lpf_taps = static_cast<size_t>(m.at("lpf_taps").asInt());
            if (m.has("tx_lpf_taps")) state.modem.tx_lpf_taps = static_cast<size_t>(m.at("tx_lpf_taps").asInt());
            if (m.has("enable_rs_outer")) state.modem.enable_rs_outer = m.at("enable_rs_outer").asBool();
            if (m.has("enable_afc"))        state.modem.enable_afc = m.at("enable_afc").asBool();
            if (m.has("enable_rx_squelch")) state.modem.enable_rx_squelch = m.at("enable_rx_squelch").asBool();
            if (m.has("enable_dc_blocker")) state.modem.enable_dc_blocker = m.at("enable_dc_blocker").asBool();
            if (m.has("rx_gain_db"))        state.modem.rx_gain_db = m.at("rx_gain_db").asFloat();
            if (m.has("agc_target_rms"))   state.modem.agc.target_rms = m.at("agc_target_rms").asFloat();
            if (m.has("agc_attack_ms"))    state.modem.agc.attack_ms  = m.at("agc_attack_ms").asFloat();
            if (m.has("agc_release_ms"))   state.modem.agc.release_ms = m.at("agc_release_ms").asFloat();
            if (m.has("agc_max_gain"))     state.modem.agc.max_gain   = m.at("agc_max_gain").asFloat();
            if (m.has("squelch_open_db"))  state.modem.squelch.open_threshold_db  = m.at("squelch_open_db").asFloat();
            if (m.has("squelch_close_db")) state.modem.squelch.close_threshold_db = m.at("squelch_close_db").asFloat();
        }

        // Hierarchical modulation config
        if (root.has("hier")) {
            auto& h = root.at("hier");
            if (h.has("enabled")) state.hier.enabled = h.at("enabled").asBool();
            if (h.has("mode"))    state.hier.mode    = static_cast<HierarchicalMode>(h.at("mode").asInt());
            if (h.has("alpha"))   state.hier.alpha   = h.at("alpha").asFloat();
            if (h.has("hp_bits")) state.hier.hp_bits = static_cast<uint8_t>(h.at("hp_bits").asInt());
            if (h.has("lp_bits")) state.hier.lp_bits = static_cast<uint8_t>(h.at("lp_bits").asInt());
        }

        // Link-budget panel inputs
        if (root.has("link_budget")) {
            auto& lb = root.at("link_budget");
            if (lb.has("tx_power_w"))  state.link_budget.tx_power_w  = lb.at("tx_power_w").asFloat();
            if (lb.has("tx_gain_db"))  state.link_budget.tx_gain_db  = lb.at("tx_gain_db").asFloat();
            if (lb.has("rx_gain_db"))  state.link_budget.rx_gain_db  = lb.at("rx_gain_db").asFloat();
            if (lb.has("cable_loss_db"))
                state.link_budget.cable_loss_db = lb.at("cable_loss_db").asFloat();
            if (lb.has("freq_mhz"))    state.link_budget.freq_mhz    = lb.at("freq_mhz").asFloat();
            if (lb.has("tx_height_m")) state.link_budget.tx_height_m = lb.at("tx_height_m").asFloat();
            if (lb.has("rx_height_m")) state.link_budget.rx_height_m = lb.at("rx_height_m").asFloat();
            if (lb.has("terrain_idx")) state.link_budget.terrain_idx = lb.at("terrain_idx").asInt();
            if (lb.has("nf_db"))       state.link_budget.nf_db       = lb.at("nf_db").asFloat();
            if (lb.has("margin_db"))   state.link_budget.margin_db   = lb.at("margin_db").asFloat();
        }

        // Alarm thresholds
        if (root.has("alarms")) {
            auto& a = root.at("alarms");
            if (a.has("snr_low_db"))       state.alarm_thresh.snr_low_db = a.at("snr_low_db").asFloat();
            if (a.has("ber_high"))         state.alarm_thresh.ber_high = a.at("ber_high").asFloat();
            if (a.has("evm_high_pct"))     state.alarm_thresh.evm_high_pct = a.at("evm_high_pct").asFloat();
            if (a.has("level_low_db"))     state.alarm_thresh.level_low_db = a.at("level_low_db").asFloat();
            if (a.has("level_high_db"))    state.alarm_thresh.level_high_db = a.at("level_high_db").asFloat();
            if (a.has("alarm_sync_loss"))  state.alarm_thresh.alarm_sync_loss = a.at("alarm_sync_loss").asBool();
            if (a.has("alarm_audio_clip")) state.alarm_thresh.alarm_audio_clip = a.at("alarm_audio_clip").asBool();
        }

        // Stream configs
        if (root.has("streams")) {
            auto& sa = root.at("streams");
            for (size_t i = 0; i < sa.arr.size() && i < MAX_STREAMS; ++i) {
                auto& sv = sa.arr[i];
                auto& sc = state.stream_configs[i];
                if (sv.has("enabled"))        sc.enabled = sv.at("enabled").asBool();
                if (sv.has("name")) {
                    const std::string& nm = sv.at("name").asStr();
                    std::strncpy(sc.name, nm.c_str(), sizeof(sc.name) - 1);
                    sc.name[sizeof(sc.name) - 1] = '\0';
                }
                if (sv.has("bitrate_bps"))    sc.bitrate_bps = static_cast<uint32_t>(sv.at("bitrate_bps").asInt());
                if (sv.has("weight"))         sc.weight = sv.at("weight").asFloat();
                if (sv.has("channels"))       sc.channels = static_cast<uint8_t>(sv.at("channels").asInt());
                if (sv.has("frame_ms"))       sc.frame_ms = static_cast<uint32_t>(sv.at("frame_ms").asInt());
                if (sv.has("sample_rate"))    sc.sample_rate = static_cast<uint32_t>(sv.at("sample_rate").asInt());
                if (sv.has("source"))         sc.source = strToStreamSource(sv.at("source").asStr());
                if (sv.has("tone_freq_hz"))   sc.tone_freq_hz = sv.at("tone_freq_hz").asFloat();
                if (sv.has("tone_amplitude")) sc.tone_amplitude = sv.at("tone_amplitude").asFloat();
                if (sv.has("file_path")) {
                    const std::string& fp = sv.at("file_path").asStr();
                    std::strncpy(sc.file_path, fp.c_str(), sizeof(sc.file_path) - 1);
                    sc.file_path[sizeof(sc.file_path) - 1] = '\0';
                }
                if (sv.has("input_device")) sc.input_device = sv.at("input_device").asInt();
                if (sv.has("opus_dred_frames"))
                    sc.opus_dred_frames = sv.at("opus_dred_frames").asInt();
                if (sv.has("app")) {
                    int av = sv.at("app").asInt();
                    if (av >= 0 && av <= 2)   // Audio/VoIP/LowDelay
                        sc.app = static_cast<OpusApplication>(av);
                }
                if (sv.has("mid_side_split"))
                    sc.mid_side_split = sv.at("mid_side_split").asFloat();
            }
        }

        // Presets
        if (root.has("presets")) {
            auto& pa = root.at("presets");
            for (size_t i = 0; i < pa.arr.size() && i < NUM_PRESETS; ++i) {
                auto& pv = pa.arr[i];
                auto& p = state.presets[i];
                if (pv.has("name"))  p.setName(pv.at("name").asStr().c_str());
                if (pv.has("valid")) p.valid = pv.at("valid").asBool();
                if (p.valid) {
                    if (pv.has("fft_size"))      p.ofdm.fft_size = static_cast<uint16_t>(pv.at("fft_size").asInt());
                    if (pv.has("modulation"))    p.ofdm.modulation = strToModulation(pv.at("modulation").asStr());
                    if (pv.has("cyclic_prefix")) p.ofdm.cyclic_prefix = strToCP(pv.at("cyclic_prefix").asStr());
                    if (pv.has("fec_rate"))      p.frame.fec_rate = strToFECRate(pv.at("fec_rate").asStr());
                    if (pv.has("sample_rate")) {
                        p.ofdm.sample_rate = static_cast<uint32_t>(pv.at("sample_rate").asInt());
                        p.modem.sample_rate = p.ofdm.sample_rate;
                    }
                    if (pv.has("center_freq"))   p.modem.center_freq = pv.at("center_freq").asFloat();
                    if (pv.has("pilot_spacing")) p.ofdm.pilot_spacing = static_cast<uint8_t>(pv.at("pilot_spacing").asInt());
                    if (pv.has("pilot_boost"))   p.ofdm.pilot_boost_db = pv.at("pilot_boost").asFloat();
                    if (pv.has("dc_null"))       p.ofdm.dc_null = pv.at("dc_null").asBool();
                    if (pv.has("guard_left"))    p.ofdm.guard_left = static_cast<uint16_t>(pv.at("guard_left").asInt());
                    if (pv.has("guard_right"))   p.ofdm.guard_right = static_cast<uint16_t>(pv.at("guard_right").asInt());
                    if (pv.has("cp_window_pct")) p.ofdm.cp_window_taper_pct = pv.at("cp_window_pct").asFloat();
                    if (pv.has("preamble_interval")) p.frame.preamble_interval = static_cast<uint32_t>(pv.at("preamble_interval").asInt());
                    if (pv.has("signal_bw"))     p.modem.signal_bw = pv.at("signal_bw").asFloat();
                    if (pv.has("lpf_taps"))      p.modem.lpf_taps = static_cast<size_t>(pv.at("lpf_taps").asInt());
                    if (pv.has("tx_lpf_taps"))   p.modem.tx_lpf_taps = static_cast<size_t>(pv.at("tx_lpf_taps").asInt());
                    if (pv.has("enable_rs_outer")) p.modem.enable_rs_outer = pv.at("enable_rs_outer").asBool();
                }
            }
        }

        // --- Validation: clamp loaded values to safe ranges so a corrupt or
        // hand-edited config can't build an invalid DSP chain. Without this,
        // fft_size=0 / sample_rate=0 / pilot_spacing=0 cause div-by-zero and
        // NaN throughout ComputedParams + the engine rebuild, and an
        // out-of-range active_preset indexes presets[] out of bounds. ---
        {
            uint16_t& fft = state.ofdm.fft_size;
            if (fft < constants::MIN_FFT_SIZE || fft > constants::MAX_FFT_SIZE ||
                popcount32(fft) != 1)
                fft = 256;
            if (state.ofdm.sample_rate == 0)  state.ofdm.sample_rate = 48000;
            if (state.modem.sample_rate == 0) state.modem.sample_rate = state.ofdm.sample_rate;
            if (state.ofdm.pilot_spacing < 1) state.ofdm.pilot_spacing = 8;
            if (state.frame.preamble_interval == 0) state.frame.preamble_interval = 50;
            if (state.ofdm.guard_left  >= fft / 2) state.ofdm.guard_left  = 0;
            if (state.ofdm.guard_right >= fft / 2) state.ofdm.guard_right = 0;
            if (state.active_preset_slot < -1 ||
                state.active_preset_slot >= static_cast<int>(NUM_PRESETS))
                state.active_preset_slot = -1;
            // hier.mode: anything that isn't a known enum value (or Custom)
            // disables hierarchy rather than feeding the mapper garbage.
            int hm = static_cast<int>(state.hier.mode);
            if (hm != static_cast<int>(HierarchicalMode::Custom) && (hm < 0 || hm > 15)) {
                state.hier.mode = HierarchicalMode::None;
                state.hier.enabled = false;
            }
            // hier.alpha is the HP/LP distance ratio (>= 1 physically; the
            // dialog clamps to [1,4]). A non-physical value (<= 0, NaN, or
            // absurdly large) feeds 20*log10(alpha) in the link-budget HP/LP
            // threshold math and yields inf/NaN readouts — reset to the default.
            if (!(state.hier.alpha >= 1.0f && state.hier.alpha <= 8.0f))
                state.hier.alpha = 2.0f;
            // Link-budget terrain index selects one of 5 combo entries
            // (FreeSpace..DenseUrban); a stale/corrupt value would crash the
            // combo setCurrentIndex / mis-map the propagation model.
            if (state.link_budget.terrain_idx < 0 ||
                state.link_budget.terrain_idx > 4)
                state.link_budget.terrain_idx = 2;
            // Center freq / signal bw must fit the passband or the
            // SoundcardModem ctor throws when the engine rebuilds — the
            // engine catches it, but the app then sits in a dead-engine
            // state from one hand-edited config value. Clamp into range.
            {
                float nyq = static_cast<float>(state.modem.sample_rate) / 2.f;
                float& fc = state.modem.center_freq;
                if (!(fc > 0.f && fc < nyq)) fc = nyq * 0.5f;
                float& bw = state.modem.signal_bw;
                float max_bw = 2.f * std::min(fc, nyq - fc) * 0.98f;
                if (!(bw >= 0.f)) bw = 0.f;              // NaN/negative → auto
                if (bw > max_bw) bw = max_bw;
            }
            // FIR tap counts: a negative JSON value cast through size_t is
            // ~1.8e19 and the filter designer would try to allocate it.
            if (state.modem.lpf_taps    > 1023) state.modem.lpf_taps    = 129;
            if (state.modem.tx_lpf_taps > 1023) state.modem.tx_lpf_taps = 129;
            // AGC / squelch numerics within the Tuning panel's ranges.
            auto clampF = [](float& v, float lo, float hi, float dflt) {
                if (!(v >= lo && v <= hi)) v = dflt;
            };
            clampF(state.modem.agc.target_rms, 0.01f, 1.0f,   0.25f);
            clampF(state.modem.agc.attack_ms,  0.1f,  500.f,  5.0f);
            clampF(state.modem.agc.release_ms, 1.f,   5000.f, 50.0f);
            clampF(state.modem.agc.max_gain,   0.f,   90.f,   60.0f);
            clampF(state.modem.squelch.open_threshold_db,  -120.f, 0.f, -50.f);
            clampF(state.modem.squelch.close_threshold_db, -120.f, 0.f, -55.f);
        }

        // Reinit spectrum freq bins
        state.spectrum.initFreqs(static_cast<float>(state.ofdm.sample_rate));

        return true;
    } catch (...) {
        return false;
    }
}

// =========================================================================
// File I/O convenience
// =========================================================================

inline bool saveConfigToFile(const AppState& state, const std::string& path) {
    std::string json = serializeConfig(state);
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f << json;
    return f.good();
}

inline bool loadConfigFromFile(const std::string& path, AppState& state) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    std::string json((std::istreambuf_iterator<char>(f)),
                      std::istreambuf_iterator<char>());
    return deserializeConfig(json, state);
}

} // namespace gw
