/*
 * ============================================================
 * STEWART PLATFORM - USB PHONE CAMERA BALL TRACKER  (C++ / OpenCV)
 * ============================================================
 * C++ port of the Python tracker. Behaviour, keys, windows, JSON files
 * and the TCP protocol to MATLAB are unchanged, so the MATLAB side and
 * existing calibration files work as before.
 *
 *   Camera (DroidCam USB, iPhone) -> HSV -> ball + plate markers
 *   MATLAB: PD control -> inverse kinematics -> RUMBA
 *
 * Lines sent to MATLAB on 127.0.0.1:<port> (default 5051), LF-terminated:
 *   CAL <scale_px_to_cm> <zero_azimuth_deg> <cx> <cy> <tracking_radius_px>
 *   REF_R <reference_radius_cm>
 *   BALL <ex_cm> <ey_cm>          ball relative to moving plate centre
 *   PLATE <dx_cm> <dy_cm>         plate centre vs saved neutral (V)
 *   LOST | PLATE_LOST | PLATE_CAL_CHANGED
 *
 * Keys (main window focused):
 *   C / R  calibrate (ball at plate centre, then click RED marker)
 *   V      save plate/base centring reference (plate centred and level)
 *   Q/Esc  quit (HSV slider values are saved)
 *
 * Command line:
 *   --list-cameras          print cameras and exit
 *   --camera-index N        pick a specific matching camera
 *   --camera-name TEXT      name filter (default "droidcam")
 *   --camera-check          run without the MATLAB TCP server
 *   --port N                TCP port (default 5051)
 *   --width W --height H    request a capture resolution (optional)
 *   --data-dir DIR          folder for the JSON files (default: exe folder)
 *
 * Build: see CMakeLists.txt (OpenCV 4.x, C++17).
 * Windows: cameras enumerated with DirectShow, captured with
 * cv::VideoCapture(index, cv::CAP_DSHOW) - same device order.
 * Linux: /sys/class/video4linux names + CAP_V4L2.
 * ============================================================
 */

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #ifndef _WIN32_WINNT
  #define _WIN32_WINNT 0x0601      // Windows 7+ (inet_ntop)
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <windows.h>
  #include <objbase.h>
  #include <oleauto.h>
  #include <dshow.h>
  #ifdef _MSC_VER
    #pragma comment(lib, "ws2_32.lib")
    #pragma comment(lib, "ole32.lib")
    #pragma comment(lib, "oleaut32.lib")
    #pragma comment(lib, "strmiids.lib")
  #endif
  typedef SOCKET sock_t;
  #define CLOSESOCK closesocket
#else
  #include <arpa/inet.h>
  #include <fcntl.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <sys/socket.h>
  #include <sys/time.h>
  #include <unistd.h>
  typedef int sock_t;
  #define INVALID_SOCKET (-1)
  #define CLOSESOCK ::close
#endif

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <thread>
#include <vector>

// ============================================================
// SETTINGS (same values as the Python version)
// ============================================================
static constexpr int    PORT = 5051;
static constexpr double BALL_DIAMETER_CM = 4.0;

static constexpr double MIN_AREA = 30;
static constexpr double MIN_RADIUS = 4;
static constexpr double MIN_CIRCULARITY = 0.45;
// CYAN (reference) = exactly the centre-to-red-marker distance.
// MAGENTA (tracking) = larger detection boundary for ball and markers.
static constexpr double REFERENCE_RADIUS_SCALE = 1.00;
static constexpr double TRACKING_RADIUS_SCALE = 1.50;
static constexpr double PLATE_EDGE_MARGIN_PX = 2.0;
static_assert(TRACKING_RADIUS_SCALE >= 1.0 && TRACKING_RADIUS_SCALE <= 3.0,
              "TRACKING_RADIUS_SCALE must be between 1.0 and 3.0");

static constexpr double SEND_HZ = 30;
static constexpr double MAX_FRAME_GAP_S = 0.6;
static constexpr double BALL_LOSS_TIMEOUT_S = 0.45;

static const char* CAL_FILE        = "calibration_config.json";
static const char* HSV_FILE        = "usb_tracker_hsv.json";
static const char* CENTER_FILE     = "platform_center_reference.json";
static const char* MARKER_HSV_FILE = "usb_marker_hsv_blue_green.json";

// Two BLUE markers on the moving plate, one GREEN marker on the base.
// Both must lie COMPLETELY inside the MAGENTA tracking circle.
static constexpr double MARKER_MIN_AREA = 14;
static constexpr double MARKER_MAX_AREA = 1500;
static constexpr double MARKER_MIN_SEPARATION_PX = 30;
static constexpr double PLATE_SEND_HZ = 30;
// Brief obstructions freeze XY correction; no fake PLATE packets.
static constexpr double MARKER_OCCLUSION_GRACE_S = 1.20;
// Lock the fixed green reference after three consistent frames.
static constexpr int    GREEN_LOCK_FRAMES = 3;
static constexpr double GREEN_LOCK_TOLERANCE_PX = 12.0;

static const double NEG_INF = -std::numeric_limits<double>::infinity();

// ============================================================
// SMALL HELPERS
// ============================================================
static double monotonicNow() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
static double pyMod(double a, double m) { return a - m * std::floor(a / m); }
static std::string lower(std::string s) {
    for (char& c : s) c = (char)std::tolower((unsigned char)c);
    return s;
}
static std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap; va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}
static cv::Point P(double x, double y) { return cv::Point(cvRound(x), cvRound(y)); }

static std::atomic<bool> g_interrupted{false};
static void onSigint(int) { g_interrupted = true; }

// ============================================================
// MINIMAL JSON (enough for the calibration files)
// ============================================================
struct Json {
    enum Type { Null, Bool, Num, Str, Arr, Obj } type = Null;
    bool b = false;
    double num = 0;
    std::string str;
    std::vector<Json> arr;              // Arr
    std::vector<std::string> keys;      // Obj
    std::vector<Json> vals;             // Obj

    const Json* get(const std::string& k) const {
        if (type != Obj) return nullptr;
        for (size_t i = 0; i < keys.size(); ++i) if (keys[i] == k) return &vals[i];
        return nullptr;
    }
    bool num_(const std::string& k, double& out) const {
        const Json* j = get(k);
        if (!j || j->type != Num || !std::isfinite(j->num)) return false;
        out = j->num; return true;
    }
    bool str_(const std::string& k, std::string& out) const {
        const Json* j = get(k);
        if (!j || j->type != Str) return false;
        out = j->str; return true;
    }
    bool pair_(const std::string& k, double& a, double& c) const {
        const Json* j = get(k);
        if (!j || j->type != Arr || j->arr.size() != 2) return false;
        if (j->arr[0].type != Num || j->arr[1].type != Num) return false;
        a = j->arr[0].num; c = j->arr[1].num;
        return std::isfinite(a) && std::isfinite(c);
    }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& s) : s_(s) {}
    bool parse(Json& out) {
        ws();
        if (!value(out)) return false;
        ws();
        return i_ == s_.size();
    }
private:
    const std::string& s_;
    size_t i_ = 0;
    void ws() { while (i_ < s_.size() && std::isspace((unsigned char)s_[i_])) ++i_; }
    bool lit(const char* w) {
        size_t n = std::strlen(w);
        if (s_.compare(i_, n, w) != 0) return false;
        i_ += n; return true;
    }
    static void utf8(std::string& o, unsigned cp) {
        if (cp < 0x80) o += (char)cp;
        else if (cp < 0x800) { o += (char)(0xC0 | (cp >> 6)); o += (char)(0x80 | (cp & 0x3F)); }
        else if (cp < 0x10000) { o += (char)(0xE0 | (cp >> 12)); o += (char)(0x80 | ((cp >> 6) & 0x3F));
                                 o += (char)(0x80 | (cp & 0x3F)); }
        else { o += (char)(0xF0 | (cp >> 18)); o += (char)(0x80 | ((cp >> 12) & 0x3F));
               o += (char)(0x80 | ((cp >> 6) & 0x3F)); o += (char)(0x80 | (cp & 0x3F)); }
    }
    bool hex4(unsigned& v) {
        if (i_ + 4 > s_.size()) return false;
        v = 0;
        for (int k = 0; k < 4; ++k) {
            char c = s_[i_++]; v <<= 4;
            if (c >= '0' && c <= '9') v |= (unsigned)(c - '0');
            else if (c >= 'a' && c <= 'f') v |= (unsigned)(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= (unsigned)(c - 'A' + 10);
            else return false;
        }
        return true;
    }
    bool string(std::string& o) {
        if (s_[i_] != '"') return false;
        ++i_;
        while (i_ < s_.size()) {
            char c = s_[i_++];
            if (c == '"') return true;
            if (c != '\\') { o += c; continue; }
            if (i_ >= s_.size()) return false;
            char e = s_[i_++];
            switch (e) {
                case '"': o += '"'; break;   case '\\': o += '\\'; break;
                case '/': o += '/'; break;   case 'b': o += '\b'; break;
                case 'f': o += '\f'; break;  case 'n': o += '\n'; break;
                case 'r': o += '\r'; break;  case 't': o += '\t'; break;
                case 'u': {
                    unsigned cp;
                    if (!hex4(cp)) return false;
                    if (cp >= 0xD800 && cp <= 0xDBFF && s_.compare(i_, 2, "\\u") == 0) {
                        i_ += 2; unsigned lo;
                        if (!hex4(lo)) return false;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    utf8(o, cp); break;
                }
                default: return false;
            }
        }
        return false;
    }
    bool value(Json& v) {
        if (i_ >= s_.size()) return false;
        char c = s_[i_];
        if (c == '{') {
            v.type = Json::Obj; ++i_; ws();
            if (i_ < s_.size() && s_[i_] == '}') { ++i_; return true; }
            for (;;) {
                ws(); std::string k;
                if (i_ >= s_.size() || !string(k)) return false;
                ws(); if (i_ >= s_.size() || s_[i_] != ':') return false; ++i_; ws();
                Json child; if (!value(child)) return false;
                v.keys.push_back(k); v.vals.push_back(std::move(child));
                ws(); if (i_ >= s_.size()) return false;
                if (s_[i_] == ',') { ++i_; continue; }
                if (s_[i_] == '}') { ++i_; return true; }
                return false;
            }
        }
        if (c == '[') {
            v.type = Json::Arr; ++i_; ws();
            if (i_ < s_.size() && s_[i_] == ']') { ++i_; return true; }
            for (;;) {
                ws(); Json child; if (!value(child)) return false;
                v.arr.push_back(std::move(child));
                ws(); if (i_ >= s_.size()) return false;
                if (s_[i_] == ',') { ++i_; continue; }
                if (s_[i_] == ']') { ++i_; return true; }
                return false;
            }
        }
        if (c == '"') { v.type = Json::Str; return string(v.str); }
        if (lit("true"))  { v.type = Json::Bool; v.b = true;  return true; }
        if (lit("false")) { v.type = Json::Bool; v.b = false; return true; }
        if (lit("null"))  { v.type = Json::Null; return true; }
        if (lit("NaN"))   { v.type = Json::Num; v.num = NAN; return true; }
        if (lit("Infinity"))  { v.type = Json::Num; v.num = INFINITY; return true; }
        if (lit("-Infinity")) { v.type = Json::Num; v.num = -INFINITY; return true; }
        const char* start = s_.c_str() + i_;
        char* end = nullptr;
        double d = std::strtod(start, &end);
        if (end == start) return false;
        i_ += (size_t)(end - start);
        v.type = Json::Num; v.num = d;
        return true;
    }
};

static std::optional<Json> loadJson(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return std::nullopt;
    std::stringstream ss; ss << f.rdbuf();
    std::string text = ss.str();
    if (text.size() >= 3 && (unsigned char)text[0] == 0xEF) text.erase(0, 3);  // UTF-8 BOM
    Json j;
    if (!JsonParser(text).parse(j)) return std::nullopt;
    return j;
}

// JSON writing helpers (output matches what Python json.dumps would read back).
static std::string jStr(const std::string& s) {
    std::string o = "\"";
    for (unsigned char c : s) {
        switch (c) {
            case '"': o += "\\\""; break;  case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;  case '\r': o += "\\r"; break;
            case '\t': o += "\\t"; break;
            default:
                if (c < 0x20) o += fmt("\\u%04x", c);
                else o += (char)c;
        }
    }
    return o + "\"";
}
static std::string jDbl(double v) {           // shortest repr that round-trips
    if (!std::isfinite(v)) return "null";
    char b[40];
    for (int p = 15; p <= 17; ++p) {
        std::snprintf(b, sizeof b, "%.*g", p, v);
        if (std::strtod(b, nullptr) == v) break;
    }
    std::string s = b;
    if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
    return s;
}
static std::string jInt(long v) { return std::to_string(v); }

static bool saveText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) { std::cerr << "WARNING: could not write " << path << "\n"; return false; }
    f << text;
    return (bool)f;
}

// ============================================================
// HSV SETTINGS + SLIDERS
// ============================================================
static const char* HSV_NAMES[6] = {"H min", "H max", "S min", "S max", "V min", "V max"};
static const int   HSV_LIMIT[6] = {179, 179, 255, 255, 255, 255};

struct Hsv {
    int v[6];
    cv::Scalar lo() const { return cv::Scalar(v[0], v[2], v[4]); }
    cv::Scalar hi() const { return cv::Scalar(v[1], v[3], v[5]); }
};
static const Hsv DEFAULT_ORANGE = {{5, 25, 100, 255, 80, 255}};      // ping pong ball
static const Hsv DEFAULT_BLUE   = {{95, 130, 110, 255, 70, 255}};
static const Hsv DEFAULT_GREEN  = {{40, 90, 95, 255, 65, 255}};

static Hsv hsvFromJson(const Json* obj, const Hsv& def) {
    Hsv h = def;
    if (!obj || obj->type != Json::Obj) return h;
    for (int k = 0; k < 6; ++k) {
        double d;
        if (obj->num_(HSV_NAMES[k], d)) h.v[k] = std::clamp((int)d, 0, HSV_LIMIT[k]);
    }
    return h;
}
static std::string hsvToJson(const Hsv& h, const std::string& indent) {
    std::string o = "{\n";
    for (int k = 0; k < 6; ++k)
        o += indent + "  " + jStr(HSV_NAMES[k]) + ": " + jInt(h.v[k]) + (k < 5 ? ",\n" : "\n");
    return o + indent + "}";
}

static void createHsvControls(const Hsv& s, const std::string& window) {
    cv::namedWindow(window, cv::WINDOW_NORMAL);
    cv::resizeWindow(window, 450, 300);
    for (int k = 0; k < 6; ++k) {
        cv::createTrackbar(HSV_NAMES[k], window, nullptr, HSV_LIMIT[k]);
        cv::setTrackbarPos(HSV_NAMES[k], window, s.v[k]);
    }
}
static bool getHsv(const std::string& window, Hsv& out) {
    for (int k = 0; k < 6; ++k) {
        int v = cv::getTrackbarPos(HSV_NAMES[k], window);
        if (v < 0) return false;                     // window was closed
        out.v[k] = v;
    }
    return true;
}

// ============================================================
// CALIBRATION
// ============================================================
struct Calibration {
    bool valid = false;
    double cx = 0, cy = 0, scale = 0, zeroAz = 0, plateRadiusPx = 0;
    bool hasRed = false;
    double redX = 0, redY = 0;
    double ballDiamCm = BALL_DIAMETER_CM;
    long w = -1, h = -1;
    std::string camera;
};

// Cyan reference radius: calibrated centre to RED reference marker.
static double referenceRadiusPx(const Calibration& c) {
    if (c.hasRed) return std::hypot(c.redX - c.cx, c.redY - c.cy) * REFERENCE_RADIUS_SCALE;
    return c.plateRadiusPx * REFERENCE_RADIUS_SCALE;
}
// OUTER tracking radius used by orange ball and blue/green markers.
static double trackingRadiusPx(const Calibration& c) {
    return referenceRadiusPx(c) * (TRACKING_RADIUS_SCALE / REFERENCE_RADIUS_SCALE);
}

static Calibration calibrationFromJson(const Json& j) {
    Calibration c;
    if (j.type != Json::Obj) return c;
    if (!j.num_("center_x", c.cx) || !j.num_("center_y", c.cy) ||
        !j.num_("scale_px_to_cm", c.scale) || !j.num_("zero_azimuth_deg", c.zeroAz))
        return c;
    c.hasRed = j.num_("red_marker_x", c.redX) && j.num_("red_marker_y", c.redY);
    bool hasPlate = j.num_("plate_radius_px", c.plateRadiusPx);
    if (!c.hasRed && !hasPlate) return c;
    j.num_("ball_diameter_cm", c.ballDiamCm);
    double d;
    if (j.num_("image_width", d))  c.w = (long)d;
    if (j.num_("image_height", d)) c.h = (long)d;
    j.str_("camera_device_name", c.camera);
    c.valid = true;
    return c;
}
static std::string calibrationToJson(const Calibration& c) {
    std::string o = "{\n";
    o += "  \"center_x\": " + jDbl(c.cx) + ",\n";
    o += "  \"center_y\": " + jDbl(c.cy) + ",\n";
    o += "  \"scale_px_to_cm\": " + jDbl(c.scale) + ",\n";
    o += "  \"zero_azimuth_deg\": " + jDbl(c.zeroAz) + ",\n";
    o += "  \"plate_radius_px\": " + jDbl(c.plateRadiusPx) + ",\n";
    o += "  \"red_marker_x\": " + jInt((long)c.redX) + ",\n";
    o += "  \"red_marker_y\": " + jInt((long)c.redY) + ",\n";
    o += "  \"ball_diameter_cm\": " + jDbl(c.ballDiamCm) + ",\n";
    o += "  \"image_width\": " + jInt(c.w) + ",\n";
    o += "  \"image_height\": " + jInt(c.h) + ",\n";
    o += "  \"camera_device_name\": " + jStr(c.camera) + "\n}";
    return o;
}

// Plate/base centring reference (V key).
struct CenterRef {
    bool valid = false;
    std::string camera;
    long w = -1, h = -1;
    double offX = 0, offY = 0;          // plate_minus_base_px
    bool hasBase = false;
    double baseX = 0, baseY = 0;        // base_marker_px
};
static CenterRef centerRefFromJson(const Json& j) {
    CenterRef r;
    if (j.type != Json::Obj) return r;
    if (!j.pair_("plate_minus_base_px", r.offX, r.offY)) return r;
    j.str_("camera_device_name", r.camera);
    double d;
    if (j.num_("image_width", d))  r.w = (long)d;
    if (j.num_("image_height", d)) r.h = (long)d;
    r.hasBase = j.pair_("base_marker_px", r.baseX, r.baseY);
    r.valid = true;
    return r;
}
static std::string centerRefToJson(const CenterRef& r) {
    std::string o = "{\n";
    o += "  \"camera_device_name\": " + jStr(r.camera) + ",\n";
    o += "  \"image_width\": " + jInt(r.w) + ",\n";
    o += "  \"image_height\": " + jInt(r.h) + ",\n";
    o += "  \"plate_minus_base_px\": [\n    " + jDbl(r.offX) + ",\n    " + jDbl(r.offY) + "\n  ],\n";
    o += "  \"base_marker_px\": [\n    " + jDbl(r.baseX) + ",\n    " + jDbl(r.baseY) + "\n  ]\n}";
    return o;
}

// ============================================================
// CAMERA DISCOVERY
// ============================================================
struct CameraInfo { int openIndex; std::string name; };

#ifdef _WIN32
static std::string wideToUtf8(const wchar_t* w) {
    int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return std::string();
    std::string s((size_t)len - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], len, nullptr, nullptr);
    return s;
}
// Same enumeration order as OpenCV's CAP_DSHOW backend uses for its index.
static std::vector<CameraInfo> enumerateCameras() {
    std::vector<CameraInfo> out;
    HRESULT init = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    ICreateDevEnum* devEnum = nullptr;
    if (SUCCEEDED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                   IID_ICreateDevEnum, (void**)&devEnum))) {
        IEnumMoniker* en = nullptr;
        if (devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &en, 0) == S_OK) {
            IMoniker* m = nullptr;
            while (en->Next(1, &m, nullptr) == S_OK) {
                std::string name = "(unknown)";
                IPropertyBag* bag = nullptr;
                if (SUCCEEDED(m->BindToStorage(nullptr, nullptr, IID_IPropertyBag, (void**)&bag))) {
                    VARIANT v; VariantInit(&v);
                    if (SUCCEEDED(bag->Read(L"FriendlyName", &v, nullptr)) && v.vt == VT_BSTR)
                        name = wideToUtf8(v.bstrVal);
                    VariantClear(&v);
                    bag->Release();
                }
                out.push_back({(int)out.size(), name});
                m->Release();
            }
            en->Release();
        }
        devEnum->Release();
    }
    if (SUCCEEDED(init)) CoUninitialize();
    return out;
}
static constexpr int CAPTURE_API = cv::CAP_DSHOW;
#elif defined(__linux__)
static std::vector<CameraInfo> enumerateCameras() {
    std::vector<CameraInfo> out;
    for (int n = 0; n < 64; ++n) {
        std::ifstream f("/sys/class/video4linux/video" + std::to_string(n) + "/name");
        if (!f) continue;
        std::string name; std::getline(f, name);
        out.push_back({n, name});
    }
    return out;
}
static constexpr int CAPTURE_API = cv::CAP_V4L2;
#else
static std::vector<CameraInfo> enumerateCameras() {   // no names available here
    std::vector<CameraInfo> out;
    for (int n = 0; n < 8; ++n) {
        cv::VideoCapture cap(n);
        if (cap.isOpened()) out.push_back({n, "Camera " + std::to_string(n)});
    }
    return out;
}
static constexpr int CAPTURE_API = cv::CAP_ANY;
#endif

static std::vector<CameraInfo> listCameras() {
    auto devices = enumerateCameras();
    std::cout << "\nAvailable cameras:\n";
    for (size_t i = 0; i < devices.size(); ++i)
        std::cout << "[" << i << "] " << devices[i].name << "\n";
    return devices;
}

// Returns the position in `devices`, or -1 (message printed).
static int findPhoneCamera(const std::vector<CameraInfo>& devices, std::optional<int> selected,
                           const std::string& filter) {
    std::vector<int> matches;
    for (size_t i = 0; i < devices.size(); ++i)
        if (lower(devices[i].name).find(filter) != std::string::npos) matches.push_back((int)i);

    if (matches.empty()) {
        std::cerr << "Phone camera ('" << filter << "') not found.\n"
                     "Connect your phone through DroidCam Desktop USB mode.\n"
                     "Refusing to use the laptop webcam.\n";
        return -1;
    }
    int index;
    if (selected) {
        if (std::find(matches.begin(), matches.end(), *selected) == matches.end()) {
            std::cerr << "Camera " << *selected << " is not a '" << filter << "' camera.\n";
            return -1;
        }
        index = *selected;
    } else {
        index = matches[0];
        for (int i : matches)
            if (lower(devices[i].name).find("video") != std::string::npos) { index = i; break; }
    }
    std::cout << "\nSelected PHONE camera: " << devices[index].name << "\n"
              << "Camera index: " << index << "\n";
    return index;
}

// Rejects empty / single-colour (blank or "no signal") frames.
static bool validFrame(const cv::Mat& f) {
    if (f.empty() || f.dims != 2 || f.depth() != CV_8U || f.channels() < 3) return false;
    const int ch = f.channels();
    double sum = 0, sum2 = 0; size_t n = 0;
    bool differs = false;
    const uchar* first = f.ptr<uchar>(0);
    for (int y = 0; y < f.rows; y += 8) {
        const uchar* row = f.ptr<uchar>(y);
        for (int x = 0; x < f.cols; x += 8) {
            const uchar* p = row + (size_t)x * ch;
            for (int c = 0; c < ch; ++c) { double v = p[c]; sum += v; sum2 += v * v; ++n; }
            if (!differs && (p[0] != first[0] || p[1] != first[1] || p[2] != first[2])) differs = true;
        }
    }
    if (n == 0) return false;
    double mean = sum / n;
    double var = std::max(0.0, sum2 / n - mean * mean);
    return std::sqrt(var) >= 8.0 && differs;
}

// ============================================================
// CAMERA READER THREAD (keeps only the newest frame)
// ============================================================
struct FrameSlot {
    std::mutex m;
    cv::Mat frame;
    double t = 0;
    bool fresh = false;
};

static void cameraWorker(int listPos, std::string filter, int reqW, int reqH,
                         FrameSlot* slot, std::atomic<bool>* stop) {
    while (!*stop) {
        try {
            auto devices = enumerateCameras();
            if (listPos >= (int)devices.size() ||
                lower(devices[listPos].name).find(filter) == std::string::npos)
                throw std::runtime_error("Selected camera is not the phone camera. "
                                         "Refusing to use laptop webcam.");
            std::cout << "Opening PHONE camera: " << devices[listPos].name << std::endl;

            cv::VideoCapture cap(devices[listPos].openIndex, CAPTURE_API);
            if (!cap.isOpened()) throw std::runtime_error("could not open device");
            if (reqW > 0) cap.set(cv::CAP_PROP_FRAME_WIDTH, reqW);
            if (reqH > 0) cap.set(cv::CAP_PROP_FRAME_HEIGHT, reqH);
            cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
            std::cout << "Phone camera capture started ("
                      << (int)cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
                      << (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT) << ")." << std::endl;

            int failures = 0;
            while (!*stop) {
                cv::Mat f;                        // fresh buffer every frame
                if (!cap.read(f) || f.empty()) {
                    if (++failures > 30) throw std::runtime_error("no frames from device");
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                failures = 0;
                if (f.channels() == 4) cv::cvtColor(f, f, cv::COLOR_BGRA2BGR);
                if (!validFrame(f)) continue;
                const double t = monotonicNow();
                std::lock_guard<std::mutex> lk(slot->m);
                slot->frame = std::move(f);
                slot->t = t;
                slot->fresh = true;
            }
        } catch (const std::exception& e) {
            std::cerr << "Phone camera capture error: " << e.what() << std::endl;
        }
        if (!*stop) {
            std::cout << "Retrying phone camera connection..." << std::endl;
            for (int k = 0; k < 100 && !*stop; ++k)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// ============================================================
// HSV BALL DETECTION
// ============================================================
struct Ball { double area, x, y, r; };
struct Dot  { double area, x, y; };

// Find the orange ball ONLY within the outer magenta tracking circle.
// Without calibration the whole frame is searched (needed for C).
static std::optional<Ball> detectBall(const cv::Mat& frame, const Hsv& s,
                                      const Calibration* cal, cv::Mat& displayMask) {
    cv::Mat blurred, hsv, mask;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, s.lo(), s.hi(), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  cv::Mat::ones(3, 3, CV_8U));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat::ones(5, 5, CV_8U));

    // Analyse the ORIGINAL full contours (clipping could make an outside
    // ball appear to be inside).
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double cx = 0, cy = 0, validR = 0;
    if (cal) {
        cx = cal->cx; cy = cal->cy;
        validR = trackingRadiusPx(*cal) - PLATE_EDGE_MARGIN_PX;
        if (!(std::isfinite(validR) && validR > MIN_RADIUS)) {
            displayMask = cv::Mat::zeros(mask.size(), CV_8U);
            return std::nullopt;
        }
        cv::Mat plate = cv::Mat::zeros(mask.size(), CV_8U);
        cv::circle(plate, P(cx, cy), std::max(1, (int)validR), cv::Scalar(255), cv::FILLED);
        cv::bitwise_and(mask, plate, displayMask);
    } else {
        displayMask = mask;
    }

    std::optional<Ball> best;
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < MIN_AREA) continue;
        cv::Point2f ctr; float r;
        cv::minEnclosingCircle(c, ctr, r);
        if (r < MIN_RADIUS) continue;
        double circ = area / std::max(CV_PI * r * r, 1.0);
        if (circ < MIN_CIRCULARITY) continue;
        // Require the full detected ball to lie inside the tracking circle.
        if (cal && std::hypot(ctr.x - cx, ctr.y - cy) + r > validR) continue;
        Ball b{area, ctr.x, ctr.y, r};
        if (!best || std::tie(b.area, b.x, b.y, b.r) > std::tie(best->area, best->x, best->y, best->r))
            best = b;
    }
    return best;
}

// (area,x,y) of complete coloured blobs INSIDE the magenta tracking circle,
// largest first. No calibration -> no markers.
static std::vector<Dot> findColorMarkers(const cv::Mat& frame, const Hsv& s,
                                         const Calibration* cal, cv::Mat& preview) {
    std::vector<Dot> pts;
    if (!cal) { preview = cv::Mat::zeros(frame.size(), CV_8U); return pts; }
    const double cx = cal->cx, cy = cal->cy;
    const double allowedR = trackingRadiusPx(*cal) - PLATE_EDGE_MARGIN_PX;
    if (!(std::isfinite(allowedR) && allowedR > 0)) {
        preview = cv::Mat::zeros(frame.size(), CV_8U); return pts;
    }
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, s.lo(), s.hi(), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8U));

    // Preview: HSV result clipped to the magenta circle.
    cv::Mat circle = cv::Mat::zeros(mask.size(), CV_8U);
    cv::circle(circle, P(cx, cy), std::max(0, (int)allowedR), cv::Scalar(255), cv::FILLED);
    cv::bitwise_and(mask, circle, preview);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < MARKER_MIN_AREA || area > MARKER_MAX_AREA) continue;
        cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        double x = m.m10 / m.m00, y = m.m01 / m.m00;
        cv::Point2f ec; float er;
        cv::minEnclosingCircle(c, ec, er);
        if (std::hypot(x - cx, y - cy) + er > allowedR) continue;
        pts.push_back({area, x, y});
    }
    std::sort(pts.begin(), pts.end(), [](const Dot& a, const Dot& b) {
        return std::tie(a.area, a.x, a.y) > std::tie(b.area, b.x, b.y);
    });
    return pts;
}

// Midpoint of the two blue markers if exactly two, far enough apart.
static std::optional<cv::Point2d> blueCenter(const std::vector<Dot>& blue) {
    if (blue.size() != 2) return std::nullopt;
    if (std::hypot(blue[0].x - blue[1].x, blue[0].y - blue[1].y) < MARKER_MIN_SEPARATION_PX)
        return std::nullopt;
    return cv::Point2d((blue[0].x + blue[1].x) / 2, (blue[0].y + blue[1].y) / 2);
}

// ============================================================
// LOCAL TCP SERVER FOR MATLAB
// ============================================================
class TcpBridge {
public:
    ~TcpBridge() { closeAll(); }

    bool start(int port) {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
        wsaUp_ = true;
#endif
        server_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (server_ == INVALID_SOCKET) return false;
        int yes = 1;
        setsockopt(server_, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof yes);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons((unsigned short)port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(server_, (sockaddr*)&a, sizeof a) != 0 || ::listen(server_, 1) != 0) {
            CLOSESOCK(server_); server_ = INVALID_SOCKET; return false;
        }
        setBlocking(server_, false);
        return true;
    }

    // Non-blocking accept; replaces any existing client.
    bool poll(std::string& who) {
        if (server_ == INVALID_SOCKET) return false;
        sockaddr_in a{};
#ifdef _WIN32
        int len = sizeof a;
#else
        socklen_t len = sizeof a;
#endif
        sock_t c = ::accept(server_, (sockaddr*)&a, &len);
        if (c == INVALID_SOCKET) return false;
        setBlocking(c, true);
        // Like Python's settimeout(0.01): a stuck MATLAB drops the link
        // instead of freezing the tracker.
#ifdef _WIN32
        DWORD ms = 10;
        setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, (const char*)&ms, sizeof ms);
#else
        timeval tv{0, 10000};
        setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);
#endif
        int one = 1;
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof one);
        if (client_ != INVALID_SOCKET) CLOSESOCK(client_);
        client_ = c;
        char ip[64] = "?";
        inet_ntop(AF_INET, &a.sin_addr, ip, sizeof ip);
        who = fmt("('%s', %d)", ip, (int)ntohs(a.sin_port));
        return true;
    }

    bool connected() const { return client_ != INVALID_SOCKET; }

    void sendLine(const std::string& msg) {
        if (client_ == INVALID_SOCKET) return;
        std::string data = msg + "\n";
        size_t off = 0;
        while (off < data.size()) {
#ifdef _WIN32
            int n = ::send(client_, data.data() + off, (int)(data.size() - off), 0);
#else
            ssize_t n = ::send(client_, data.data() + off, data.size() - off, MSG_NOSIGNAL);
#endif
            if (n <= 0) { CLOSESOCK(client_); client_ = INVALID_SOCKET; return; }
            off += (size_t)n;
        }
    }

    void closeAll() {
        if (client_ != INVALID_SOCKET) { CLOSESOCK(client_); client_ = INVALID_SOCKET; }
        if (server_ != INVALID_SOCKET) { CLOSESOCK(server_); server_ = INVALID_SOCKET; }
#ifdef _WIN32
        if (wsaUp_) { WSACleanup(); wsaUp_ = false; }
#endif
    }

private:
    sock_t server_ = INVALID_SOCKET, client_ = INVALID_SOCKET;
#ifdef _WIN32
    bool wsaUp_ = false;
#endif
    static void setBlocking(sock_t s, bool blocking) {
#ifdef _WIN32
        u_long mode = blocking ? 0 : 1;
        ioctlsocket(s, FIONBIO, &mode);
#else
        int fl = fcntl(s, F_GETFL, 0);
        fcntl(s, F_SETFL, blocking ? (fl & ~O_NONBLOCK) : (fl | O_NONBLOCK));
#endif
    }
};

// ============================================================
// MAIN TRACKER
// ============================================================
static const char* WIN_MAIN   = "Stewart USB Ball Tracker";
static const char* WIN_ORANGE = "Orange Mask";
static const char* WIN_BLUE   = "Blue Mask";
static const char* WIN_GREEN  = "Green Mask";
static const char* HSV_ORANGE = "HSV Tuning";
static const char* HSV_BLUE   = "Blue Marker HSV";
static const char* HSV_GREEN  = "Green Marker HSV";

struct Options {
    std::optional<int> cameraIndex;
    bool listOnly = false, cameraCheck = false;
    int port = PORT, width = 0, height = 0;
    std::string cameraFilter = "droidcam";
    std::string dataDir;
};

class Tracker {
public:
    Tracker(const Options& o, std::string cameraName)
        : opt_(o), cameraName_(std::move(cameraName)) {}

    int run(int listPos);

private:
    Options opt_;
    std::string cameraName_;
    std::string path(const char* f) const { return opt_.dataDir + "/" + f; }

    Calibration cal_;
    CenterRef centerRef_;
    bool newCalibration_ = false;

    // Calibration click state (C/R then click the red marker).
    bool waitingForRed_ = false;
    cv::Point2d pendCenter_;
    double pendRadius_ = 0;
    cv::Size pendShape_;

    std::optional<cv::Point2d> lockedBase_, greenCandidate_;
    int greenCount_ = 0;
    std::optional<std::pair<cv::Point2d, cv::Point2d>> detected_;   // (plate, base)
    std::optional<cv::Point2d> lastGoodPlatformCenter_;
    double lastPlateSent_ = NEG_INF, lastPlateGood_ = NEG_INF;
    bool plateLostSent_ = false;

    cv::Mat currentFrame_;
    std::optional<Ball> latestBall_;
    std::vector<Dot> blueDots_, greenDots_;
    double lastFrameTime_ = NEG_INF, lastDetectionTime_ = NEG_INF, lastSendTime_ = NEG_INF;
    bool lostSent_ = false;
    double lastWarning_ = NEG_INF;

    TcpBridge bridge_;

    void send(const std::string& s) { bridge_.sendLine(s); }
    void sendCalibration();
    void processFrame(double frameTime);
    void handleKeys(int key, double now, bool& quit);
    void saveHsvFiles();

    static void onMouse(int event, int x, int y, int, void* ud) {
        if (event == cv::EVENT_LBUTTONDOWN) static_cast<Tracker*>(ud)->onClick(x, y);
    }
    void onClick(int x, int y);
};

void Tracker::sendCalibration() {
    if (!cal_.valid) return;
    send(fmt("CAL %.8f %.8f %.3f %.3f %.4f", cal_.scale, cal_.zeroAz, cal_.cx, cal_.cy,
             trackingRadiusPx(cal_)));
    // Physical R (cm) = CYAN radius (px) x 4-cm-ball scale. Distinct from
    // CAL's larger MAGENTA ball-detection radius.
    send(fmt("REF_R %.8f", referenceRadiusPx(cal_) * cal_.scale));
}

void Tracker::onClick(int x, int y) {
    if (!waitingForRed_) return;
    const double cx = pendCenter_.x, cy = pendCenter_.y;
    const double plateRadius = std::hypot(x - cx, y - cy);
    if (plateRadius <= 10) { std::cout << "Click the RED marker away from center.\n"; return; }

    const double heading = pyMod(std::atan2(x - cx, cy - y) * 180.0 / CV_PI, 360.0);
    Calibration c;
    c.valid = true;
    c.cx = cx; c.cy = cy;
    c.scale = BALL_DIAMETER_CM / (2 * pendRadius_);
    c.zeroAz = pyMod(-heading, 360.0);
    c.plateRadiusPx = plateRadius;
    c.hasRed = true; c.redX = x; c.redY = y;
    c.ballDiamCm = BALL_DIAMETER_CM;
    c.w = pendShape_.width; c.h = pendShape_.height;
    c.camera = cameraName_;
    cal_ = c;
    waitingForRed_ = false;
    saveText(path(CAL_FILE), calibrationToJson(cal_));
    newCalibration_ = true;
    std::cout << fmt("Calibration saved. Active plate radius: %.1fpx (cyan 1.00x, tracking %.2fx).\n",
                     trackingRadiusPx(cal_), TRACKING_RADIUS_SCALE);
}

void Tracker::processFrame(double frameTime) {
    cv::Mat& frame = currentFrame_;
    const int width = frame.cols, height = frame.rows;

    // Previous calibration invalid if the size changes.
    if (cal_.valid && (cal_.w != width || cal_.h != height)) {
        cal_.valid = false;
        centerRef_.valid = false;
        lockedBase_.reset(); greenCandidate_.reset(); greenCount_ = 0;
        newCalibration_ = false;
        send("LOST");
        send("PLATE_LOST");
        std::cout << "Camera resolution changed. Calibrate again.\n";
    }
    const Calibration* cal = cal_.valid ? &cal_ : nullptr;

    Hsv orange = DEFAULT_ORANGE, blue = DEFAULT_BLUE, green = DEFAULT_GREEN;
    getHsv(HSV_ORANGE, orange);
    getHsv(HSV_BLUE, blue);
    getHsv(HSV_GREEN, green);

    cv::Mat mask, blueMask, greenMask;
    std::optional<Ball> ball = detectBall(frame, orange, cal, mask);
    latestBall_ = ball;
    cv::Mat display = frame.clone();

    // Moving plate: green is only needed until locked.
    blueDots_  = findColorMarkers(frame, blue, cal, blueMask);
    greenDots_ = findColorMarkers(frame, green, cal, greenMask);

    // Lock the green base after three consecutive, consistent measurements
    // taken while both blue markers are visible.
    if (!lockedBase_ && cal) {
        if (greenDots_.size() == 1 && blueDots_.size() == 2) {
            cv::Point2d cand(greenDots_[0].x, greenDots_[0].y);
            if (greenCandidate_ &&
                std::hypot(cand.x - greenCandidate_->x, cand.y - greenCandidate_->y) <= GREEN_LOCK_TOLERANCE_PX)
                ++greenCount_;
            else
                greenCount_ = 1;
            greenCandidate_ = cand;
            if (greenCount_ >= GREEN_LOCK_FRAMES) {
                lockedBase_ = cand;
                std::cout << fmt("GREEN base marker LOCKED at (%.1f, %.1f) px\n", cand.x, cand.y);
            }
        } else {
            greenCandidate_.reset();
            greenCount_ = 0;
        }
    }

    // Platform measurement = two blue markers + locked green reference.
    const std::optional<cv::Point2d> visibleBlueCenter = blueCenter(blueDots_);
    detected_.reset();
    if (lockedBase_ && visibleBlueCenter) detected_ = std::make_pair(*visibleBlueCenter, *lockedBase_);

    for (const Dot& d : blueDots_) {
        cv::circle(display, P(d.x, d.y), 12, cv::Scalar(255, 0, 0), 2);
        cv::circle(display, P(d.x, d.y), 3, cv::Scalar(255, 255, 255), -1);
    }
    for (const Dot& d : greenDots_) {
        cv::circle(display, P(d.x, d.y), 12, cv::Scalar(0, 255, 0), 2);
        cv::circle(display, P(d.x, d.y), 3, cv::Scalar(255, 255, 255), -1);
    }

    const bool markerValid = detected_.has_value();
    std::string status = fmt("BLUE: %d/2 | GREEN: ", (int)blueDots_.size()) +
        (lockedBase_ ? std::string("LOCKED") : std::to_string(greenDots_.size()) + "/1");
    cv::putText(display, status, {12, 137}, cv::FONT_HERSHEY_SIMPLEX, 0.67,
                markerValid ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255), 2);
    if (lockedBase_)
        cv::drawMarker(display, P(lockedBase_->x, lockedBase_->y), cv::Scalar(0, 180, 0),
                       cv::MARKER_CROSS, 28, 2);
    if (!cal)
        cv::putText(display, "CALIBRATE CYAN CIRCLE FIRST (C)", {12, 166},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 140, 255), 2);
    else if (!markerValid)
        cv::putText(display, "Need 2 BLUE + locked GREEN reference", {12, 166},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 140, 255), 2);

    if (detected_) {
        const cv::Point2d plate = detected_->first, base = detected_->second;
        lastGoodPlatformCenter_ = plate;
        cv::drawMarker(display, P(plate.x, plate.y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
        cv::drawMarker(display, P(base.x, base.y), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
        if (centerRef_.valid && cal && centerRef_.w == width && centerRef_.h == height) {
            const double dx = plate.x - base.x - centerRef_.offX;
            const double dy = plate.y - base.y - centerRef_.offY;
            const double ex = dx * cal_.scale, ey = dy * cal_.scale;
            lastPlateGood_ = frameTime;
            if (bridge_.connected() && frameTime - lastPlateSent_ >= 1.0 / PLATE_SEND_HZ) {
                send(fmt("PLATE %.6f %.6f", ex, ey));
                lastPlateSent_ = frameTime;
                plateLostSent_ = false;
            }
            cv::Point target = P(base.x + centerRef_.offX, base.y + centerRef_.offY);
            cv::drawMarker(display, target, cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 23, 2);
            cv::line(display, P(plate.x, plate.y), target, cv::Scalar(0, 0, 255), 2);
            cv::putText(display, fmt("Plate offset: %.2f cm", std::hypot(ex, ey)), {12, 105},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        }
    } else if (centerRef_.valid) {
        const double age = frameTime - lastPlateGood_;
        if (lastGoodPlatformCenter_ && age >= 0 && age < MARKER_OCCLUSION_GRACE_S) {
            cv::drawMarker(display, P(lastGoodPlatformCenter_->x, lastGoodPlatformCenter_->y),
                           cv::Scalar(180, 180, 180), cv::MARKER_CROSS, 20, 2);
            cv::putText(display, fmt("MARKER OCCLUDED %.2fs - XY FROZEN", age), {12, 105},
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 200, 255), 2);
        } else {
            cv::putText(display, "MARKERS LOST - MATLAB HOLD", {12, 105},
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        }
    }

    // Plate ROI circles: cyan = reference (1.00x), magenta = tracking.
    if (cal) {
        cv::circle(display, P(cal_.cx, cal_.cy), cvRound(referenceRadiusPx(cal_)),
                   cv::Scalar(255, 255, 0), 2);
        cv::circle(display, P(cal_.cx, cal_.cy), cvRound(trackingRadiusPx(cal_)),
                   cv::Scalar(255, 0, 255), 2);
        if (!detected_)
            cv::drawMarker(display, P(cal_.cx, cal_.cy), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 20, 2);
        if (cal_.hasRed)
            cv::drawMarker(display, P(cal_.redX, cal_.redY), cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2);
    }

    if (ball) {
        cv::circle(display, P(ball->x, ball->y), cvRound(ball->r), cv::Scalar(0, 255, 0), 2);
        cv::circle(display, P(ball->x, ball->y), 3, cv::Scalar(0, 0, 255), -1);

        if (cal) {
            const bool graceOk = lastGoodPlatformCenter_ &&
                                 frameTime - lastPlateGood_ >= 0 &&
                                 frameTime - lastPlateGood_ < MARKER_OCCLUSION_GRACE_S;
            cv::Point2d centre;
            if (detected_)              centre = detected_->first;
            else if (visibleBlueCenter) centre = *visibleBlueCenter;
            else if (graceOk)           centre = *lastGoodPlatformCenter_;   // frozen estimate
            else                        centre = cv::Point2d(cal_.cx, cal_.cy);
            const double ex = (ball->x - centre.x) * cal_.scale;
            const double ey = (ball->y - centre.y) * cal_.scale;

            lastDetectionTime_ = frameTime;
            const bool movingCentreFresh = detected_.has_value() || graceOk;
            // With a centring reference, BALL needs a recent moving-plate centre.
            const bool maySend = !centerRef_.valid || movingCentreFresh;
            if (maySend && bridge_.connected() && frameTime - lastSendTime_ >= 1.0 / SEND_HZ) {
                send(fmt("BALL %.6f %.6f", ex, ey));
                lastSendTime_ = frameTime;
                lostSent_ = false;
            }
            cv::putText(display, fmt("Ball X=%+.2f cm Y=%+.2f cm", ex, ey), {12, 32},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }
    }

    std::string message;
    if (waitingForRed_)  message = "Click RED reference marker";
    else if (!cal)       message = "Ball at CENTER: press C";
    else message = fmt("CYAN %.2fx | TRACK (magenta) %.2fx", REFERENCE_RADIUS_SCALE, TRACKING_RADIUS_SCALE);
    cv::putText(display, message, {12, 70}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);

    cv::imshow(WIN_MAIN, display);
    cv::imshow(WIN_ORANGE, mask);
    cv::imshow(WIN_BLUE, blueMask);
    cv::imshow(WIN_GREEN, greenMask);
}

void Tracker::handleKeys(int key, double now, bool& quit) {
    if (key == 27 || key == 'q' || key == 'Q') { quit = true; return; }

    if (key == 'v' || key == 'V') {
        if (!detected_ || now - lastFrameTime_ >= MAX_FRAME_GAP_S) {
            std::cout << "Cannot calibrate center: lock GREEN and detect both BLUE markers first.\n";
        } else if (!cal_.valid) {
            std::cout << "Calibrate the ball center first (C), then center reference (V).\n";
        } else if (greenDots_.size() != 1) {
            std::cout << "GREEN must be visible when saving/recalibrating with V.\n";
        } else {
            // Only press V with the upper plate physically centred and level.
            const cv::Point2d plate = detected_->first;
            const cv::Point2d base(greenDots_[0].x, greenDots_[0].y);
            lockedBase_ = base;
            greenCandidate_ = base;
            greenCount_ = GREEN_LOCK_FRAMES;
            CenterRef r;
            r.valid = true;
            r.camera = cameraName_;
            r.w = currentFrame_.cols; r.h = currentFrame_.rows;
            r.offX = plate.x - base.x; r.offY = plate.y - base.y;
            r.hasBase = true; r.baseX = base.x; r.baseY = base.y;
            centerRef_ = r;
            saveText(path(CENTER_FILE), centerRefToJson(r));
            std::cout << "Plate/base reference saved, GREEN position locked across restarts.\n";
            send("PLATE_CAL_CHANGED");
            lastPlateGood_ = NEG_INF;
            lastGoodPlatformCenter_.reset();
            plateLostSent_ = true;
        }
    }

    if (key == 'c' || key == 'C' || key == 'r' || key == 'R') {
        if (!currentFrame_.empty() && latestBall_ && now - lastFrameTime_ < MAX_FRAME_GAP_S) {
            waitingForRed_ = true;
            pendCenter_ = cv::Point2d(latestBall_->x, latestBall_->y);
            pendRadius_ = latestBall_->r;
            pendShape_ = currentFrame_.size();

            cal_.valid = false;
            centerRef_.valid = false;
            lockedBase_.reset(); greenCandidate_.reset(); greenCount_ = 0;
            lastGoodPlatformCenter_.reset();
            lastPlateGood_ = NEG_INF;

            send("LOST");
            send("PLATE_CAL_CHANGED");
            std::cout << "Ball center stored. Click the RED reference marker.\n";
        } else {
            std::cout << "Ball not detected. Adjust HSV sliders.\n";
        }
    }
}

void Tracker::saveHsvFiles() {
    Hsv o, b, g;
    if (getHsv(HSV_ORANGE, o)) saveText(path(HSV_FILE), hsvToJson(o, ""));
    if (getHsv(HSV_BLUE, b) && getHsv(HSV_GREEN, g))
        saveText(path(MARKER_HSV_FILE),
                 "{\n  \"blue\": " + hsvToJson(b, "  ") + ",\n  \"green\": " + hsvToJson(g, "  ") + "\n}");
}

int Tracker::run(int listPos) {
    // ---- HSV sliders ----
    auto hsvJson = loadJson(path(HSV_FILE));
    createHsvControls(hsvFromJson(hsvJson ? &*hsvJson : nullptr, DEFAULT_ORANGE), HSV_ORANGE);
    auto markerJson = loadJson(path(MARKER_HSV_FILE));
    const Json* mj = markerJson ? &*markerJson : nullptr;
    createHsvControls(hsvFromJson(mj ? mj->get("blue") : nullptr, DEFAULT_BLUE), HSV_BLUE);
    createHsvControls(hsvFromJson(mj ? mj->get("green") : nullptr, DEFAULT_GREEN), HSV_GREEN);

    cv::namedWindow(WIN_MAIN, cv::WINDOW_NORMAL);
    cv::namedWindow(WIN_ORANGE, cv::WINDOW_NORMAL);
    cv::namedWindow(WIN_BLUE, cv::WINDOW_NORMAL);
    cv::namedWindow(WIN_GREEN, cv::WINDOW_NORMAL);

    // ---- Calibration ----
    if (auto j = loadJson(path(CAL_FILE))) {
        Calibration c = calibrationFromJson(*j);
        if (c.valid && c.camera == cameraName_) {
            cal_ = c;
            std::cout << "Loaded calibration: " << path(CAL_FILE) << "\n"
                      << fmt("Reference radius: %.1f px; tracking radius: %.1f px (scale=%.2f).\n",
                             referenceRadiusPx(cal_), trackingRadiusPx(cal_), TRACKING_RADIUS_SCALE);
        }
    }
    std::cout << "Tracker data folder: " << opt_.dataDir << "\n";

    // Independent of the ball-centre calibration; create it only with the
    // upper plate physically centred and level.
    if (auto j = loadJson(path(CENTER_FILE))) {
        CenterRef r = centerRefFromJson(*j);
        if (r.valid && r.camera == cameraName_) centerRef_ = r;
    }
    if (centerRef_.valid)
        std::cout << "Plate-base centering reference loaded; press V to recapture at neutral.\n";
    // Saved green base position survives restarts (same camera + resolution).
    if (centerRef_.valid && cal_.valid && centerRef_.hasBase &&
        centerRef_.w == cal_.w && centerRef_.h == cal_.h) {
        lockedBase_ = cv::Point2d(centerRef_.baseX, centerRef_.baseY);
        std::cout << fmt("GREEN base position restored from saved V reference: (%.1f, %.1f)\n",
                         lockedBase_->x, lockedBase_->y);
    }

    cv::setMouseCallback(WIN_MAIN, &Tracker::onMouse, this);

    // ---- TCP server ----
    if (!opt_.cameraCheck) {
        if (!bridge_.start(opt_.port)) {
            std::cerr << "Could not open MATLAB bridge on 127.0.0.1:" << opt_.port
                      << " (port in use? Python tracker still running?)\n";
            return 1;
        }
        std::cout << "MATLAB bridge: 127.0.0.1:" << opt_.port << "\n";
    }

    std::cout << "\nUSB phone camera tracker running.\n"
                 "C = Calibrate (ball must be at plate center)\n"
                 "R = Recalibrate (changes require MATLAB manual re-arm)\n"
                 "V = Save PLATE/BASE centering reference when physically centered\n"
                 "Markers: 2 BLUE on upper plate, 1 GREEN on fixed base (locks after detection)\n"
                 "Blue Marker HSV and Green Marker HSV windows tune them separately.\n"
                 "Only markers COMPLETELY INSIDE the magenta tracking circle are accepted.\n"
                 "After GREEN is locked, only the two BLUE markers require continuous tracking.\n"
              << fmt("BLUE occlusions: XY trim frozen for up to %.2fs; longer gaps -> HOLD.\n",
                     MARKER_OCCLUSION_GRACE_S)
              << "Q = Quit\n" << std::endl;

    // ---- Camera thread ----
    FrameSlot slot;
    std::atomic<bool> stop{false};
    std::thread reader(cameraWorker, listPos, opt_.cameraFilter, opt_.width, opt_.height, &slot, &stop);

    bool quit = false;
    while (!quit && !g_interrupted) {
        const double now = monotonicNow();

        std::string who;
        if (bridge_.poll(who)) {
            std::cout << "MATLAB connected: " << who << std::endl;
            sendCalibration();
            lostSent_ = false;
        }

        bool newFrame = false;
        double frameTime = 0;
        {
            std::lock_guard<std::mutex> lk(slot.m);
            if (slot.fresh) {
                currentFrame_ = std::move(slot.frame);
                slot.frame = cv::Mat();
                frameTime = slot.t;
                slot.fresh = false;
                newFrame = true;
            }
        }
        if (newFrame) {
            lastFrameTime_ = frameTime;
            processFrame(frameTime);
        }

        if (newCalibration_) { newCalibration_ = false; sendCalibration(); }

        // Ball loss.
        if (cal_.valid && now - lastDetectionTime_ > BALL_LOSS_TIMEOUT_S && !lostSent_) {
            send("LOST");
            lostSent_ = true;
        }
        // Stale plate markers: MATLAB must stop centring correction.
        if (centerRef_.valid && now - lastPlateGood_ >= MARKER_OCCLUSION_GRACE_S && !plateLostSent_) {
            send("PLATE_LOST");
            plateLostSent_ = true;
        }
        // Camera disconnect.
        if (now - lastFrameTime_ > MAX_FRAME_GAP_S && now - lastWarning_ > 2) {
            std::cout << "USB camera frame gap. MATLAB should HOLD." << std::endl;
            lastWarning_ = now;
        }

        const int key = cv::waitKey(1);
        if (key >= 0) handleKeys(key & 0xFF, now, quit);

        if (currentFrame_.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    stop = true;
    if (reader.joinable()) reader.join();
    saveHsvFiles();
    bridge_.closeAll();
    cv::destroyAllWindows();
    return 0;
}

// ============================================================
// ENTRY POINT
// ============================================================
static std::string executableDir(const char* argv0) {
#ifdef _WIN32
    char buf[MAX_PATH];
    DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    std::string p = (n > 0 && n < MAX_PATH) ? std::string(buf, n) : std::string(argv0);
#elif defined(__linux__)
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof buf - 1);
    std::string p = n > 0 ? std::string(buf, (size_t)n) : std::string(argv0);
#else
    std::string p = argv0;
#endif
    size_t k = p.find_last_of("/\\");
    return k == std::string::npos ? std::string(".") : p.substr(0, k);
}

static void usage() {
    std::cout << "Usage: stewart_tracker [--list-cameras] [--camera-index N] [--camera-name TEXT]\n"
                 "                       [--camera-check] [--port N] [--width W --height H]\n"
                 "                       [--data-dir DIR]\n";
}

int main(int argc, char** argv) {
    Options opt;
    opt.dataDir = executableDir(argv[0]);
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) { std::cerr << what << " needs a value\n"; std::exit(2); }
            return argv[++i];
        };
        try {
            if (a == "--camera-index")      opt.cameraIndex = std::stoi(next("--camera-index"));
            else if (a == "--list-cameras") opt.listOnly = true;
            else if (a == "--camera-check") opt.cameraCheck = true;
            else if (a == "--port")         opt.port = std::stoi(next("--port"));
            else if (a == "--camera-name")  opt.cameraFilter = lower(next("--camera-name"));
            else if (a == "--width")        opt.width = std::stoi(next("--width"));
            else if (a == "--height")       opt.height = std::stoi(next("--height"));
            else if (a == "--data-dir")     opt.dataDir = next("--data-dir");
            else if (a == "-h" || a == "--help") { usage(); return 0; }
            else { std::cerr << "Unknown argument: " << a << "\n"; usage(); return 2; }
        } catch (const std::exception&) {
            std::cerr << "Bad value for " << a << "\n"; return 2;
        }
    }
    std::signal(SIGINT, onSigint);

    auto devices = listCameras();
    if (opt.listOnly) return 0;

    const int listPos = findPhoneCamera(devices, opt.cameraIndex, opt.cameraFilter);
    if (listPos < 0) return 1;

    Tracker tracker(opt, devices[listPos].name);
    return tracker.run(listPos);
}
