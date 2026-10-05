from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "commissioning" / "simplefoc_motor_app_main.cpp"
RUNTIME = ROOT / "main" / "runtime_simplefoc_motor_commissioning.cpp"
BACKEND = ROOT / "components" / "triwhirl_simplefoc" / "simplefoc_motor_backend.cpp"
BACKEND_HEADER = (
    ROOT
    / "components"
    / "triwhirl_simplefoc"
    / "include"
    / "triwhirl"
    / "simplefoc_motor_backend.hpp"
)
PLATFORMIO = ROOT / "platformio.ini"
CMAKE = ROOT / "main" / "CMakeLists.txt"

app = APP.read_text(encoding="utf-8")
runtime = RUNTIME.read_text(encoding="utf-8")
backend = BACKEND.read_text(encoding="utf-8")
backend_header = BACKEND_HEADER.read_text(encoding="utf-8")
platformio = PLATFORMIO.read_text(encoding="utf-8")
cmake = CMAKE.read_text(encoding="utf-8")

required_app = [
    '#include "esp32-hal-alloc-ble-mem.h"',
    'CONFIG_BT_NIMBLE_ENABLED',
    'CONFIG_BT_NIMBLE_ROLE_PERIPHERAL',
    'runSimpleFocMotorCommissioning()',
]
required_runtime = [
    '#include "triwhirl/ble_transport.hpp"',
    'ble::init()',
    'ble::read(',
    'ble::writeBlocking(',
    'transport=ble_gatt,name=%s,owner=SimpleFOC',
    'actuator=DISABLED,autostart=0',
    'kMaxCommissioningVoltageV = 0.5F',
    'kMaxTargetVelocityRadS = 5.0F',
    'kMaxDriveDurationMs = 1500U',
    'begin_stage=%s',
    'backend->beginFailureStage()',
    'xQueueOverwrite(result_queue, &result)',
]
required_backend = [
    'SimpleFocMotorBeginFailureStage::kSensorPath',
    'SimpleFocMotorBeginFailureStage::kDriverInit',
    'SimpleFocMotorBeginFailureStage::kMotorInit',
    'SimpleFocMotorBeginFailureStage::kInitFoc',
    'Keep begin diagnostics as bounded in-memory state',
]
required_backend_header = [
    'enum class SimpleFocMotorBeginFailureStage',
    'simpleFocMotorBeginFailureStageName(',
    'beginFailureStage() const',
]
required_platformio = [
    '[env:simplefoc-motor-commissioning-ble]',
    'board_build.esp-idf.sdkconfig_path = .pio/build/simplefoc-motor-commissioning-ble/sdkconfig',
    '-DSDKCONFIG_DEFAULTS="sdkconfig.defaults"',
    '-I.pio/libdeps/simplefoc-motor-commissioning-ble/Arduino-FOC/src',
]
required_cmake = [
    'if(TRIWHIRL_ROUTE_B_MOTOR_COMMISSIONING)',
    'triwhirl_ble',
]
forbidden = [
    '[env:simplefoc-motor-commissioning-bt]',
    'esp32-hal-alloc-bt-classic-mem.h',
    'btStartMode(BT_MODE_CLASSIC_BT)',
    'esp_spp_',
    'bluetooth_spp_direct',
]
backend_forbidden = [
    'SimpleFOCDebug',
    'Serial.begin(',
    'Serial.println(',
]

missing = [token for token in required_app if token not in app]
missing += [token for token in required_runtime if token not in runtime]
missing += [token for token in required_backend if token not in backend]
missing += [token for token in required_backend_header if token not in backend_header]
missing += [token for token in required_platformio if token not in platformio]
missing += [token for token in required_cmake if token not in cmake]
present_forbidden = [
    token
    for token in forbidden
    if token in app or token in runtime or token in platformio
]
present_forbidden += [token for token in backend_forbidden if token in backend]
if missing or present_forbidden:
    details = []
    if missing:
        details.append("missing: " + ", ".join(missing))
    if present_forbidden:
        details.append("forbidden: " + ", ".join(present_forbidden))
    raise SystemExit("SimpleFOC motor commissioning contract violation; " + "; ".join(details))

print("SimpleFOC motor commissioning BLE-GATT contract: PASS")
