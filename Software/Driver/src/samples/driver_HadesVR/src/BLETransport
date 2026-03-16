// UARTTransport.cpp

#include <windows.h>
#include <string>
#include <sstream>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <optional>

#include "UARTTransport.hpp"
#include "driverlog.h"
#include "settingsAPIKeys.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Storage.Streams.h>

using namespace winrt;
using namespace Windows::Foundation;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace Windows::Storage::Streams;

#define FRAME_MAGIC             ((uint8_t)0xAA)
#define FRAME_HEADER_SIZE       (3u)  // Magic + seqno + size

#define FRAME_MAGIC_INDEX       (0u)
#define FRAME_SEQNO_INDEX       (1u)
#define FRAME_DATA_SIZE_INDEX   (2u)

static constexpr int ASSEMBLE_TIMEOUT_MS = 25; // wait for partner packet this long before forcing send
static constexpr int RECONNECT_DELAY_MS = 1500;
static constexpr int CONNECT_TIMEOUT_MS = 8000;

// Helper: parse MAC "01:23:45:67:89:AB" -> uint64_t
static uint64_t ParseMacAddress(const std::string& macStr) {
    uint64_t mac = 0;
    std::istringstream iss(macStr);
    std::string hexByte;
    while (std::getline(iss, hexByte, ':')) {
        if (hexByte.empty()) continue;
        uint64_t b = std::stoull(hexByte, nullptr, 16) & 0xFF;
        mac = (mac << 8) | b;
    }
    return mac;
}

// Parse UUID string to winrt::guid (accepts forms with or without braces)
static winrt::guid ParseUUID(const std::string& uuidStr) {
    std::string s = uuidStr;
    if (s.front() != '{') {
        s = "{" + s + "}";
    }
    std::wstring w(s.begin(), s.end());
    return winrt::guid(w.c_str());
}

// Raw packet from controller (sent by ESP32 peripheral)
#pragma pack(push,1)
struct CtrlRawPacket {
    uint8_t ControllerID; // 1 or 2
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;
    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
    uint16_t Buttons;
    uint8_t Trigger;
    int8_t AxisX;
    int8_t AxisY;
    uint8_t vBat;
    // total = 1 + 2*6 +2 +1 +1 +1 +1 = 1 + 12 + 2 + 1 + 1 + 1 + 1 = 19 bytes (align 19)
};
static_assert(sizeof(CtrlRawPacket) <= 32, "CtrlRawPacket too big");
#pragma pack(pop)

// ControllerPacket layout expected by existing driver (packed exactly as in original receiver)
#pragma pack(push,1)
struct ControllerPacket {
    uint8_t PacketID; // = 2

    // Controller 1
    int16_t Ctrl1_QuatW;
    int16_t Ctrl1_QuatX;
    int16_t Ctrl1_QuatY;
    int16_t Ctrl1_QuatZ;
    int16_t Ctrl1_AccelX;
    int16_t Ctrl1_AccelY;
    int16_t Ctrl1_AccelZ;
    uint16_t Ctrl1_Buttons;
    uint8_t Ctrl1_Trigger;
    int8_t  Ctrl1_axisX;
    int8_t  Ctrl1_axisY;
    int8_t  Ctrl1_trackY;
    uint8_t Ctrl1_vBat;
    uint8_t Ctrl1_THUMB;
    uint8_t Ctrl1_INDEX;
    uint8_t Ctrl1_MIDDLE;
    uint8_t Ctrl1_RING;
    uint8_t Ctrl1_PINKY;
    uint8_t Ctrl1_AnalogGrip;
    uint16_t Ctrl1_Data; // padding / misc

    // Controller 2
    int16_t Ctrl2_QuatW;
    int16_t Ctrl2_QuatX;
    int16_t Ctrl2_QuatY;
    int16_t Ctrl2_QuatZ;
    int16_t Ctrl2_AccelX;
    int16_t Ctrl2_AccelY;
    int16_t Ctrl2_AccelZ;
    uint16_t Ctrl2_Buttons;
    uint8_t Ctrl2_Trigger;
    int8_t  Ctrl2_axisX;
    int8_t  Ctrl2_axisY;
    int8_t  Ctrl2_trackY;
    uint8_t Ctrl2_vBat;
    uint8_t Ctrl2_THUMB;
    uint8_t Ctrl2_INDEX;
    uint8_t Ctrl2_MIDDLE;
    uint8_t Ctrl2_RING;
    uint8_t Ctrl2_PINKY;
    uint8_t Ctrl2_AnalogGrip;
    uint16_t Ctrl2_Data;
};
static_assert(sizeof(ControllerPacket) <= 256, "ControllerPacket too large");
#pragma pack(pop)

// Convert raw IMU to "steamVR-style" ControllerPacket fields.
// We intentionally store quats as zeros here because we assume Madgwick will be done on PC.
// We'll fill quaternion fields with reasonable placeholders (identity) so driver doesn't get NaNs.
static void ConvertRawToControllerPacket(const CtrlRawPacket& a, const CtrlRawPacket& b, ControllerPacket& out) {
    // Packet ID
    out.PacketID = 2;

    // Controller 1 - map accelerations and buttons
    // Quaternions are set to identity (1,0,0,0) scaled to int16 for compatibility
    out.Ctrl1_QuatW = (int16_t)(32767); // W = 1.0
    out.Ctrl1_QuatX = 0;
    out.Ctrl1_QuatY = 0;
    out.Ctrl1_QuatZ = 0;

    out.Ctrl1_AccelX = a.AccX;
    out.Ctrl1_AccelY = a.AccY;
    out.Ctrl1_AccelZ = a.AccZ;

    out.Ctrl1_Buttons = a.Buttons;
    out.Ctrl1_Trigger = a.Trigger;
    out.Ctrl1_axisX = a.AxisX;
    out.Ctrl1_axisY = a.AxisY;
    out.Ctrl1_trackY = 0; // not used
    out.Ctrl1_vBat = a.vBat;

    // fingers/analog grip placeholders
    out.Ctrl1_THUMB = 0;
    out.Ctrl1_INDEX = 0;
    out.Ctrl1_MIDDLE = 0;
    out.Ctrl1_RING = 0;
    out.Ctrl1_PINKY = 0;
    out.Ctrl1_AnalogGrip = 0;
    out.Ctrl1_Data = 0xFFFF;

    // Controller 2
    out.Ctrl2_QuatW = (int16_t)(32767);
    out.Ctrl2_QuatX = 0;
    out.Ctrl2_QuatY = 0;
    out.Ctrl2_QuatZ = 0;

    out.Ctrl2_AccelX = b.AccX;
    out.Ctrl2_AccelY = b.AccY;
    out.Ctrl2_AccelZ = b.AccZ;

    out.Ctrl2_Buttons = b.Buttons;
    out.Ctrl2_Trigger = b.Trigger;
    out.Ctrl2_axisX = b.AxisX;
    out.Ctrl2_axisY = b.AxisY;
    out.Ctrl2_trackY = 0;
    out.Ctrl2_vBat = b.vBat;

    out.Ctrl2_THUMB = 0;
    out.Ctrl2_INDEX = 0;
    out.Ctrl2_MIDDLE = 0;
    out.Ctrl2_RING = 0;
    out.Ctrl2_PINKY = 0;
    out.Ctrl2_AnalogGrip = 0;
    out.Ctrl2_Data = 0xFFFF;
}

// BLE context
struct BLEContext {
    BluetoothLEDevice device{ nullptr };
    GattDeviceService service{ nullptr };
    GattCharacteristic characteristic{ nullptr };
    winrt::event_token valueChangedToken{};
    bool initialized{ false };
    IAsyncOperation<GattCommunicationStatus> requestMtuOp{ nullptr };
    GattSession session{ nullptr };
};

static BLEContext g_ble;
static std::mutex g_ringbuf_mutex;
static std::atomic<uint8_t> g_outSeq{0};

// State holder for assembling two controllers
struct CtrlState {
    std::optional<CtrlRawPacket> c1;
    std::optional<CtrlRawPacket> c2;
    std::chrono::steady_clock::time_point t1{};
    std::chrono::steady_clock::time_point t2{};
    std::mutex m;
    void reset() {
        std::lock_guard<std::mutex> lk(m);
        c1.reset(); c2.reset();
    }
} g_ctrlState;

static std::atomic<bool> g_stopMonitor{ false };
static std::thread g_monitorThread;

// Forward declaration
void EnqueueAssembledControllerPacketAndSignal(const ControllerPacket& pkt);

// UARTTransport constructor assumed defined in UARTTransport.hpp
UARTTransport::UARTTransport()
    : connected(false),
      lastSeqno(0),
      hasReceivedFirstFrame(false),
      hSerial(NULL)
{
    ring_buffer_init(&ringbuffer);
}

// Internal: attempt synchronous connect (called from Start)
static bool ConnectToDevice(const std::string& macAddressStr, const winrt::guid& svcGuid, const winrt::guid& chrGuid, BLEContext& outCtx) {
    try {
        uint64_t mac = ParseMacAddress(macAddressStr);
        // Ensure COM apartment is initialized for this thread
        init_apartment(apartment_type::single_threaded);

        auto deviceOp = BluetoothLEDevice::FromBluetoothAddressAsync(mac);
        auto device = deviceOp.get();
        if (!device) {
            DriverLog("[UARTTransport] Error: FromBluetoothAddressAsync returned null device");
            return false;
        }

        // Create session to control connection parameters
        GattSession session = GattSession::FromDeviceIdAsync(device.DeviceId()).get();
        if (session) {
            // request lower latency by setting DesiredConnectionInterval - not all stacks honor it.
            // This is a best-effort; WinRT doesn't expose direct CI manipulation reliably across adapters.
            session.MinConnectionIntervalInMilliseconds(7); // best-effort
            session.MaxConnectionIntervalInMilliseconds(15);
        }

        // get services
        auto servicesRes = device.GetGattServicesForUuidAsync(svcGuid).get();
        if (servicesRes.Status() != GattCommunicationStatus::Success || servicesRes.Services().Size() == 0) {
            DriverLog("[UARTTransport] Error: Service not found on device");
            return false;
        }
        auto service = servicesRes.Services().GetAt(0);

        // characteristics
        auto charsRes = service.GetCharacteristicsForUuidAsync(chrGuid).get();
        if (charsRes.Status() != GattCommunicationStatus::Success || charsRes.Characteristics().Size() == 0) {
            DriverLog("[UARTTransport] Error: Characteristic not found in service");
            return false;
        }
        auto characteristic = charsRes.Characteristics().GetAt(0);

        outCtx.device = device;
        outCtx.service = service;
        outCtx.characteristic = characteristic;
        outCtx.session = session;
        outCtx.initialized = true;

        DriverLog("[UARTTransport] Connected and located service/characteristic");
        return true;
    } catch (winrt::hresult_error const& ex) {
        DriverLog("[UARTTransport] WinRT error during connect: %ls", ex.message().c_str());
        return false;
    } catch (std::exception const& ex) {
        DriverLog("[UARTTransport] C++ exception during connect: %s", ex.what());
        return false;
    } catch (...) {
        DriverLog("[UARTTransport] Unknown error during connect");
        return false;
    }
}

// Subscribe and set callback
static bool SubscribeToNotifications(BLEContext& ctx, std::function<void(const IBuffer&)> onValue) {
    try {
        if (!ctx.characteristic) {
            DriverLog("[UARTTransport] Subscribe failed: no characteristic");
            return false;
        }

        // Request notifications
        auto setRes = ctx.characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
            GattClientCharacteristicConfigurationDescriptorValue::Notify).get();
        if (setRes != GattCommunicationStatus::Success) {
            DriverLog("[UARTTransport] Error: Unable to enable notifications (status=%d)", (int)setRes);
            return false;
        }

        // Register callback (capture by value)
        ctx.valueChangedToken = ctx.characteristic.ValueChanged(
            [onValue](GattCharacteristic const&, GattValueChangedEventArgs const& args) {
                onValue(args.CharacteristicValue());
            });

        DriverLog("[UARTTransport] Subscribed to notifications");
        return true;
    } catch (winrt::hresult_error const& ex) {
        DriverLog("[UARTTransport] WinRT error during subscribe: %ls", ex.message().c_str());
        return false;
    } catch (std::exception const& ex) {
        DriverLog("[UARTTransport] C++ error during subscribe: %s", ex.what());
        return false;
    } catch (...) {
        DriverLog("[UARTTransport] Unknown error during subscribe");
        return false;
    }
}

// Unsubscribe & clear
static void UnsubscribeAndClear(BLEContext& ctx) {
    try {
        if (ctx.initialized && ctx.characteristic) {
            // remove handler
            try {
                ctx.characteristic.ValueChanged(ctx.valueChangedToken);
            } catch (...) { /* ignore */ }

            // disable notifications
            try {
                ctx.characteristic.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue::None).get();
            } catch (...) { /* ignore */ }
        }
    } catch (...) {
        // ignore gracefully
    }

    try {
        if (ctx.device) {
            ctx.device.Close();
            ctx.device = nullptr;
        }
    } catch (...) { /*ignore*/ }

    ctx.characteristic = nullptr;
    ctx.service = nullptr;
    ctx.initialized = false;
    ctx.valueChangedToken = {};
    ctx.session = nullptr;
}

// Enqueue assembled packet into ring buffer (thread-safe)
void EnqueueAssembledControllerPacketAndSignal(const ControllerPacket& pkt) {
    uint8_t frame[FRAME_HEADER_SIZE + sizeof(ControllerPacket)];
    frame[0] = FRAME_MAGIC;
    uint8_t seq = (uint8_t)(g_outSeq.fetch_add(1) & 0xFF);
    frame[1] = seq;
    frame[2] = (uint8_t)sizeof(ControllerPacket);

    // copy payload after header
    std::memcpy(frame + FRAME_HEADER_SIZE, &pkt, sizeof(ControllerPacket));

    {
        std::lock_guard<std::mutex> lk(g_ringbuf_mutex);
        ring_buffer_queue_arr(&ringbuffer, frame, (ring_buffer_size_t)(FRAME_HEADER_SIZE + sizeof(ControllerPacket)));
    }
}

// Handles raw buffer from BLE (called from WinRT callback)
static void HandleBleBuffer(const IBuffer& buf) {
    try {
        uint32_t len = buf.Length();
        if (len == 0) return;

        // Copy bytes out quickly
        std::vector<uint8_t> bytes;
        bytes.resize(len);
        DataReader::FromBuffer(buf).ReadBytes(winrt::array_view<uint8_t>(bytes.data(), bytes.size()));

        // Expect CtrlRawPacket from ESP32 peripheral
        if (bytes.size() < sizeof(CtrlRawPacket)) {
            // ignore small packets
            return;
        }

        // safe copy into struct
        CtrlRawPacket pkt{};
        std::memcpy(&pkt, bytes.data(), sizeof(CtrlRawPacket));

        auto now = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lk(g_ctrlState.m);
            if (pkt.ControllerID == 1) {
                g_ctrlState.c1 = pkt;
                g_ctrlState.t1 = now;
            } else if (pkt.ControllerID == 2) {
                g_ctrlState.c2 = pkt;
                g_ctrlState.t2 = now;
            } else {
                // unknown id: ignore
                return;
            }

            // If both present and timestamps close enough -> assemble
            if (g_ctrlState.c1 && g_ctrlState.c2) {
                auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(g_ctrlState.t1 - g_ctrlState.t2).count();
                if (std::abs(diff) <= ASSEMBLE_TIMEOUT_MS) {
                    ControllerPacket outPkt;
                    ConvertRawToControllerPacket(*g_ctrlState.c1, *g_ctrlState.c2, outPkt);
                    EnqueueAssembledControllerPacketAndSignal(outPkt);
                    // reset states after assembly
                    g_ctrlState.c1.reset();
                    g_ctrlState.c2.reset();
                    return;
                }
            }

            // Otherwise, if only one present, we'll wait for partner; but if the other is stale, force-send later by monitor thread.
        }
    } catch (...) {
        // swallow to avoid crashing WinRT callback thread
    }
}

// Monitor thread: ensures connection is alive, and forces combined sends if one controller times out waiting for the other.
static void MonitorThreadProc(std::string macStr, winrt::guid svcGuid, winrt::guid chrGuid) {
    // initialize apartment for this thread
    init_apartment(apartment_type::single_threaded);

    while (!g_stopMonitor.load()) {
        if (!g_ble.initialized) {
            // attempt connect
            if (!ConnectToDevice(macStr, svcGuid, chrGuid, g_ble)) {
                DriverLog("[UARTTransport] Connect attempt failed; retrying in %d ms", RECONNECT_DELAY_MS);
                std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                continue;
            }

            // subscribe
            bool ok = SubscribeToNotifications(g_ble, [](const IBuffer& buf){ HandleBleBuffer(buf); });
            if (!ok) {
                UnsubscribeAndClear(g_ble);
                std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                continue;
            }

            DriverLog("[UARTTransport] BLE subscription active");
        }

        // If we have only one controller waiting too long, force-assemble with default/zeros to avoid blocking driver forever.
        {
            std::lock_guard<std::mutex> lk(g_ctrlState.m);
            auto now = std::chrono::steady_clock::now();
            if (g_ctrlState.c1 && !g_ctrlState.c2) {
                auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_ctrlState.t1).count();
                if (age >= ASSEMBLE_TIMEOUT_MS) {
                    // assemble with zeros for c2
                    CtrlRawPacket zero{};
                    ControllerPacket outPkt;
                    ConvertRawToControllerPacket(*g_ctrlState.c1, zero, outPkt);
                    EnqueueAssembledControllerPacketAndSignal(outPkt);
                    g_ctrlState.c1.reset();
                }
            } else if (g_ctrlState.c2 && !g_ctrlState.c1) {
                auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_ctrlState.t2).count();
                if (age >= ASSEMBLE_TIMEOUT_MS) {
                    CtrlRawPacket zero{};
                    ControllerPacket outPkt;
                    ConvertRawToControllerPacket(zero, *g_ctrlState.c2, outPkt);
                    EnqueueAssembledControllerPacketAndSignal(outPkt);
                    g_ctrlState.c2.reset();
                }
            }
        }

        // Basic health check: confirm connection still alive by checking device properties
        try {
            if (g_ble.initialized && g_ble.device) {
                // Simple query; if throws, assume lost connection
                auto name = g_ble.device.Name();
                // No-op, just keep alive
            }
        } catch (...) {
            DriverLog("[UARTTransport] BLE device seems disconnected; cleaning up");
            UnsubscribeAndClear(g_ble);
            connected = false;
            // allow reconnect loop
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } // while

    // cleanup on thread exit
    UnsubscribeAndClear(g_ble);
}

// Public API: Start
int UARTTransport::Start()
{
    char macAddress[128] = {0};
    char serviceUuidStr[128] = {0};
    char charUuidStr[128] = {0};
    vr::EVRSettingsError error;

    // Read settings (assume keys exist and type is string)
    vr::VRSettings()->GetString(k_pch_Driver_Section, k_pch_BT_DeviceAddress, macAddress, sizeof(macAddress));
    vr::VRSettings()->GetString(k_pch_Driver_Section, k_pch_BT_ServiceUUID, serviceUuidStr, sizeof(serviceUuidStr));
    vr::VRSettings()->GetString(k_pch_Driver_Section, k_pch_BT_CharacteristicUUID, charUuidStr, sizeof(charUuidStr));

    std::string macStr(macAddress);
    std::string svcStr(serviceUuidStr);
    std::string chrStr(charUuidStr);

    if (macStr.empty() || svcStr.empty() || chrStr.empty()) {
        DriverLog("[UARTTransport] Error: Missing BLE configuration settings");
        return 1;
    }

    try {
        // init apartment for caller thread
        init_apartment(apartment_type::multi_threaded);
    } catch (...) {
        // ignore if already initialized
    }

    winrt::guid svcGuid = ParseUUID(svcStr);
    winrt::guid chrGuid = ParseUUID(chrStr);

    // start monitor thread (it will attempt connections and subscribe)
    g_stopMonitor.store(false);
    try {
        g_monitorThread = std::thread([macStr, svcGuid, chrGuid]() {
            MonitorThreadProc(macStr, svcGuid, chrGuid);
        });
    } catch (const std::exception& ex) {
        DriverLog("[UARTTransport] Error creating monitor thread: %s", ex.what());
        return 1;
    }

    // wait a short time for connection to become ready
    auto startT = std::chrono::steady_clock::now();
    const int waitMs = CONNECT_TIMEOUT_MS;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startT).count() < waitMs) {
        if (g_ble.initialized && g_ble.characteristic) {
            connected = true;
            DriverLog("[UARTTransport] BLE Start: connected");
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // if not connected yet, return success anyway because monitor thread will keep trying
    DriverLog("[UARTTransport] BLE Start: connection pending (monitor thread active)");
    return 0;
}

// Public API: Stop
void UARTTransport::Stop()
{
    DriverLog("[UARTTransport] Stopping BLE transport...");
    g_stopMonitor.store(true);

    try {
        if (g_monitorThread.joinable()) {
            g_monitorThread.join();
        }
    } catch (...) {
        // ignore
    }

    // cleanup BLE context
    UnsubscribeAndClear(g_ble);

    // reset ring buffer & states
    {
        std::lock_guard<std::mutex> lk(g_ringbuf_mutex);
        ring_buffer_init(&ringbuffer);
    }
    g_ctrlState.reset();

    connected = false;
    DriverLog("[UARTTransport] BLE transport stopped");
}

bool UARTTransport::IsConnected()
{
    return connected;
}

// Called by BLE callback via HandleBleBuffer -> EnqueueAssembled...
void UARTTransport::OnBLEData(uint8_t* data, size_t len)
{
    // Not used in this variant because we use HandleBleBuffer wrapper
    std::lock_guard<std::mutex> lk(g_ringbuf_mutex);
    ring_buffer_queue_arr(&ringbuffer, data, (ring_buffer_size_t)len);
}

// ReadPacket: unchanged parsing logic except bugfix for header size check and safer dequeues
int UARTTransport::ReadPacket(uint8_t* outBuf, size_t length)
{
    std::lock_guard<std::mutex> lk(g_ringbuf_mutex);

    while (true) {
        // Drop bytes until magic found
        uint8_t magic;
        while (ring_buffer_peek(&ringbuffer, &magic, FRAME_MAGIC_INDEX)) {
            if (magic == FRAME_MAGIC) {
                break;
            }
            // drop single byte
            ring_buffer_dequeue(&ringbuffer, &magic);
        }

        // Need header
        const ring_buffer_size_t totalSize = ring_buffer_num_items(&ringbuffer);
        if (totalSize < FRAME_HEADER_SIZE) {
            break;
        }

        // Peek data size byte at index FRAME_DATA_SIZE_INDEX
        uint8_t dataSize = 0u;
        ring_buffer_peek(&ringbuffer, &dataSize, FRAME_DATA_SIZE_INDEX);

        // Correct check: header size + payload
        if (totalSize >= (FRAME_HEADER_SIZE + (ring_buffer_size_t)dataSize)) {
            // seqno check
            const uint8_t expectedSeqno = (lastSeqno + 1) & UINT8_MAX;
            uint8_t currentSeqno = 0;
            ring_buffer_peek(&ringbuffer, &currentSeqno, FRAME_SEQNO_INDEX);

            if (!hasReceivedFirstFrame) {
                hasReceivedFirstFrame = true;
            }
            else if (currentSeqno != expectedSeqno) {
                DriverLog("[UARTTransport] Expected seqno 0x%02x, got 0x%02x", expectedSeqno, currentSeqno);
            }

            lastSeqno = currentSeqno;

            // Dequeue header into local buffer (we don't need header in outBuf)
            uint8_t header[FRAME_HEADER_SIZE];
            ring_buffer_dequeue_arr(&ringbuffer, header, FRAME_HEADER_SIZE);

            // Now dequeue payload into outBuf (size = dataSize)
            ring_buffer_dequeue_arr(&ringbuffer, outBuf, (ring_buffer_size_t)dataSize);

            if (length != dataSize) {
                DriverLog("[UARTTransport] Warning: Expected %zu bytes, got %d bytes", length, dataSize);
                return 0;
            }

            return (int)dataSize;
        }

        // Buffer overrun handling
        if (totalSize == UART_BUFFER_SIZE) {
            DriverLog("[UARTTransport] Warning: buffer overrun");
            ring_buffer_init(&ringbuffer);
        }

        break;
    }

    return 0;
}
