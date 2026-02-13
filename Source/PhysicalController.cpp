/***************************************************************************************************
 * Xidi
 * DirectInput interface for XInput controllers.
 ***************************************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2026
 ***********************************************************************************************//**
 * @file PhysicalController.cpp
 * Implementation of all functionality for communicating with physical controllers.
 **************************************************************************************************/

#include "PhysicalController.h"

#include <cstdint>
#include <mutex>
#include <set>
#include <stop_token>
#include <string>
#include <thread>

#include <Infra/Core/Message.h>

#include "ApiWindows.h"
#include "ConcurrencyWrapper.h"
#include "ControllerTypes.h"
#include "ForceFeedbackDevice.h"
#include "Globals.h"
#include "ImportApiWinMM.h"
#include "ImportApiXInput.h"
#include "Mapper.h"
#include "Strings.h"
#include "VirtualController.h"

#include <hidsdi.h>
#include <hidusage.h>
#include <hidpi.h>
#include <hidclass.h>
#include <setupapi.h>

const int XINPUT_GAMEPAD_GUIDE = 0x0400;
const int XINPUT_GAMEPAD_SHARE = 0x0800;

namespace Xidi
{
  namespace Controller
  {
    /// Raw physical state data for each of the possible physical controllers.
    static ConcurrencyWrapper<SPhysicalState> physicalControllerState[kPhysicalControllerCount];

    /// State data for each of the possible physical controllers after it is passed through a mapper
    /// but without any further processing.
    static ConcurrencyWrapper<SState> rawVirtualControllerState[kPhysicalControllerCount];

    /// Per-controller force feedback device buffer objects.
    static ForceFeedback::Device* physicalControllerForceFeedbackBuffer;

    /// Pointers to the virtual controller objects registered for force feedback with each physical
    /// controller.
    static std::set<const VirtualController*>
        physicalControllerForceFeedbackRegistration[kPhysicalControllerCount];

    /// Mutex objects for protecting against concurrent accesses to the physical controller force
    /// feedback registration data.
    static std::mutex physicalControllerForceFeedbackMutex[kPhysicalControllerCount];

    /// Shared share button states for each controller, updated by HID threads.
    static ConcurrencyWrapper<bool> shareButtonState[kPhysicalControllerCount];

    /// HID device handles for each controller.
    /// All elements explicitly initialized to INVALID_HANDLE_VALUE.
    static HANDLE hHidDevice[kPhysicalControllerCount] = {
        INVALID_HANDLE_VALUE,
        INVALID_HANDLE_VALUE,
        INVALID_HANDLE_VALUE,
        INVALID_HANDLE_VALUE};

    static std::thread hidReadThread[kPhysicalControllerCount];

    /// Mutex and set for tracking which HID device paths have already been claimed
    /// by a controller thread, so that each controller opens a DIFFERENT HID device.
    static std::mutex hidDeviceClaimMutex;
    static std::set<std::wstring> claimedHidDevicePaths;

    /// Computes an opaque source identifier from a given controller identifier.
    static inline uint32_t OpaqueControllerSourceIdentifier(
        TControllerIdentifier controllerIdentifier)
    {
      return (uint32_t)controllerIdentifier;
    }

    /// Background thread to poll HID for share button.
    /// This thread is the sole owner of the HID handle for its controller index.
    /// No other thread should close or modify hHidDevice[controllerIdentifier].
    /// @param [in] controllerIdentifier Identifier of the controller to poll.
    static void PollHidForShareButton(TControllerIdentifier controllerIdentifier)
    {
      // Track the device path this controller claimed, so we can unclaim on disconnect
      std::wstring myClaimedPath;

      // Stagger initial startup by controller index to ensure deterministic
      // enumeration order so HID devices are assigned to matching XInput indices.
      Sleep(controllerIdentifier * 500);

      while (true)
      {
        Sleep(1); // Poll at 1000Hz

        if (hHidDevice[controllerIdentifier] == INVALID_HANDLE_VALUE)
        {
          // Only search for HID devices if our XInput controller is actually
          // connected. This is critical: without this check, threads for
          // controllers that have no physical hardware (e.g. controllers 2
          // and 3 when only 2 gamepads are connected) would perpetually
          // enumerate at 1ms intervals and steal HID devices that belong to
          // other controllers during reconnection events.
          XINPUT_STATE xinputCheck;
          if (ImportApiXInput::XInputGetState(controllerIdentifier, &xinputCheck) != ERROR_SUCCESS)
            continue;

          GUID hidGuid;
          HidD_GetHidGuid(&hidGuid);

          HDEVINFO hDevInfo = SetupDiGetClassDevs(
              &hidGuid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

          if (hDevInfo != INVALID_HANDLE_VALUE)
          {
            SP_DEVICE_INTERFACE_DATA interfaceData = {};
            interfaceData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

            for (DWORD index = 0;
                 SetupDiEnumDeviceInterfaces(
                     hDevInfo, nullptr, &hidGuid, index, &interfaceData);
                 ++index)
            {
              DWORD requiredSize = 0;
              SetupDiGetDeviceInterfaceDetail(
                  hDevInfo, &interfaceData, nullptr, 0, &requiredSize, nullptr);

              PSP_DEVICE_INTERFACE_DETAIL_DATA detailData =
                  (PSP_DEVICE_INTERFACE_DETAIL_DATA) new BYTE[requiredSize];
              detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

              if (SetupDiGetDeviceInterfaceDetail(
                      hDevInfo, &interfaceData, detailData, requiredSize, nullptr, nullptr))
              {
                // Check if this device path is already claimed by another controller
                {
                  std::lock_guard<std::mutex> lock(hidDeviceClaimMutex);
                  if (claimedHidDevicePaths.count(detailData->DevicePath) > 0)
                  {
                    delete[] detailData;
                    continue;
                  }
                }

                HANDLE hDevice = CreateFile(
                    detailData->DevicePath,
                    GENERIC_READ,
                    FILE_SHARE_READ | FILE_SHARE_WRITE,
                    nullptr,
                    OPEN_EXISTING,
                    0,
                    nullptr);

                if (hDevice != INVALID_HANDLE_VALUE)
                {
                  HIDD_ATTRIBUTES attributes = {};
                  attributes.Size = sizeof(HIDD_ATTRIBUTES);

                  if (HidD_GetAttributes(hDevice, &attributes))
                  {
                    if (attributes.VendorID == 0x045E)
                    {
                      if (attributes.ProductID == 0x02FD || attributes.ProductID == 0x0B13 ||
                          attributes.ProductID == 0x02FF)
                      {
                        // Verify this is the correct HID collection by checking
                        // input report byte length. Only the 16-byte report
                        // collection contains the Share button data.
                        bool correctCollection = false;
                        PHIDP_PREPARSED_DATA preparsed = nullptr;
                        if (HidD_GetPreparsedData(hDevice, &preparsed))
                        {
                          HIDP_CAPS caps = {};
                          if (HidP_GetCaps(preparsed, &caps) == HIDP_STATUS_SUCCESS)
                          {
                            if (caps.InputReportByteLength == 16)
                              correctCollection = true;
                          }
                          HidD_FreePreparsedData(preparsed);
                        }

                        if (correctCollection)
                        {
                          // Claim this device path
                          {
                            std::lock_guard<std::mutex> lock(hidDeviceClaimMutex);
                            claimedHidDevicePaths.insert(detailData->DevicePath);
                            myClaimedPath = detailData->DevicePath;
                          }

                          hHidDevice[controllerIdentifier] = hDevice;

                          Infra::Message::OutputFormatted(
                              Infra::Message::ESeverity::Info,
                              L"Opened HID device for controller %u with PID 0x%04X.",
                              controllerIdentifier,
                              attributes.ProductID);

                          delete[] detailData;
                          break;
                        }
                      }
                    }
                  }
                  CloseHandle(hDevice);
                }
              }
              delete[] detailData;
            }

            SetupDiDestroyDeviceInfoList(hDevInfo);
          }
        }

        if (hHidDevice[controllerIdentifier] != INVALID_HANDLE_VALUE)
        {
          BYTE reportBuffer[32] = {};
          DWORD bytesRead = 0;

          if (ReadFile(
                  hHidDevice[controllerIdentifier],
                  reportBuffer,
                  sizeof(reportBuffer),
                  &bytesRead,
                  nullptr))
          {
            bool sharePressed = (reportBuffer[0] == 0x00 && (reportBuffer[12] & 0x08));
            shareButtonState[controllerIdentifier].Update(sharePressed);
          }
          else
          {
            // Any ReadFile failure means the device is gone or in a bad state.
            // Close the handle and unclaim the path so re-enumeration can occur.
            CloseHandle(hHidDevice[controllerIdentifier]);
            hHidDevice[controllerIdentifier] = INVALID_HANDLE_VALUE;

            // Clear share button state immediately on disconnect
            shareButtonState[controllerIdentifier].Update(false);

            // Unclaim the path so it can be re-claimed on reconnect
            if (!myClaimedPath.empty())
            {
              std::lock_guard<std::mutex> lock(hidDeviceClaimMutex);
              claimedHidDevicePaths.erase(myClaimedPath);
              myClaimedPath.clear();
            }

            Infra::Message::OutputFormatted(
                Infra::Message::ESeverity::Info,
                L"HID device lost for controller %u, will re-enumerate on reconnect.",
                controllerIdentifier);

            // Stagger reconnection backoff by controller index so that
            // lower-indexed controllers enumerate and claim first if
            // multiple controllers reconnect simultaneously.
            Sleep(200 + controllerIdentifier * 200);
          }
        }
      }
    }

    /// Reads physical controller state.
    /// Note: HID handle lifecycle is managed exclusively by PollHidForShareButton.
    static SPhysicalState ReadPhysicalControllerState(TControllerIdentifier controllerIdentifier)
    {
      XINPUT_STATE xinputState;
      DWORD xinputGetStateResult =
          ImportApiXInput::XInputGetState(controllerIdentifier, &xinputState);

      switch (xinputGetStateResult)
      {
        case ERROR_SUCCESS:
          if (shareButtonState[controllerIdentifier].Get())
            xinputState.Gamepad.wButtons |= XINPUT_GAMEPAD_SHARE;

          return {
              .deviceStatus = EPhysicalDeviceStatus::Ok,
              .stick =
                  {xinputState.Gamepad.sThumbLX,
                   xinputState.Gamepad.sThumbLY,
                   xinputState.Gamepad.sThumbRX,
                   xinputState.Gamepad.sThumbRY},
              .trigger = {xinputState.Gamepad.bLeftTrigger, xinputState.Gamepad.bRightTrigger},
              .button = (uint16_t)(xinputState.Gamepad.wButtons)};

        case ERROR_DEVICE_NOT_CONNECTED:
          return {.deviceStatus = EPhysicalDeviceStatus::NotConnected};

        default:
          return {.deviceStatus = EPhysicalDeviceStatus::Error};
      }
    }

    static_assert(1u << (unsigned int)EPhysicalButton::DpadUp == XINPUT_GAMEPAD_DPAD_UP);
    static_assert(1u << (unsigned int)EPhysicalButton::DpadDown == XINPUT_GAMEPAD_DPAD_DOWN);
    static_assert(1u << (unsigned int)EPhysicalButton::DpadLeft == XINPUT_GAMEPAD_DPAD_LEFT);
    static_assert(1u << (unsigned int)EPhysicalButton::DpadRight == XINPUT_GAMEPAD_DPAD_RIGHT);
    static_assert(1u << (unsigned int)EPhysicalButton::Start == XINPUT_GAMEPAD_START);
    static_assert(1u << (unsigned int)EPhysicalButton::Back == XINPUT_GAMEPAD_BACK);
    static_assert(1u << (unsigned int)EPhysicalButton::LS == XINPUT_GAMEPAD_LEFT_THUMB);
    static_assert(1u << (unsigned int)EPhysicalButton::RS == XINPUT_GAMEPAD_RIGHT_THUMB);
    static_assert(1u << (unsigned int)EPhysicalButton::LB == XINPUT_GAMEPAD_LEFT_SHOULDER);
    static_assert(1u << (unsigned int)EPhysicalButton::RB == XINPUT_GAMEPAD_RIGHT_SHOULDER);
    static_assert(1u << (unsigned int)EPhysicalButton::Guide == XINPUT_GAMEPAD_GUIDE);
    static_assert(1u << (unsigned int)EPhysicalButton::Share == XINPUT_GAMEPAD_SHARE);
    static_assert(1u << (unsigned int)EPhysicalButton::A == XINPUT_GAMEPAD_A);
    static_assert(1u << (unsigned int)EPhysicalButton::B == XINPUT_GAMEPAD_B);
    static_assert(1u << (unsigned int)EPhysicalButton::X == XINPUT_GAMEPAD_X);
    static_assert(1u << (unsigned int)EPhysicalButton::Y == XINPUT_GAMEPAD_Y);

    static ForceFeedback::TPhysicalActuatorValue ScaledVibrationStrength(
        ForceFeedback::TPhysicalActuatorValue vibrationStrength, double scalingFactor)
    {
      if (0.0 == scalingFactor)
        return 0;
      else if (1.0 == scalingFactor)
        return static_cast<ForceFeedback::TPhysicalActuatorValue>(vibrationStrength);

      constexpr double kMaxVibrationStrength =
          static_cast<double>(std::numeric_limits<ForceFeedback::TPhysicalActuatorValue>::max());

      const double scaledVibrationStrength = static_cast<double>(vibrationStrength) * scalingFactor;
      return static_cast<ForceFeedback::TPhysicalActuatorValue>(
          std::min(scaledVibrationStrength, kMaxVibrationStrength));
    }

    static bool WritePhysicalControllerVibration(
        TControllerIdentifier controllerIdentifier,
        ForceFeedback::SPhysicalActuatorComponents vibration)
    {
      static const double kForceFeedbackEffectStrengthScalingFactor =
          static_cast<double>(
              Globals::GetConfigurationData()
                  [Strings::kStrConfigurationSectionProperties]
                  [Strings::kStrConfigurationSettingPropertiesForceFeedbackEffectStrengthPercent]
                      .ValueOr(100)) /
          100.0;

      XINPUT_VIBRATION xinputVibration = {
          .wLeftMotorSpeed = ScaledVibrationStrength(
              vibration.leftMotor, kForceFeedbackEffectStrengthScalingFactor),
          .wRightMotorSpeed = ScaledVibrationStrength(
              vibration.rightMotor, kForceFeedbackEffectStrengthScalingFactor)};

      return (
          ERROR_SUCCESS ==
          ImportApiXInput::XInputSetState((DWORD)controllerIdentifier, &xinputVibration));
    }

    static void ForceFeedbackActuateEffects(TControllerIdentifier controllerIdentifier)
    {
      constexpr ForceFeedback::TOrderedMagnitudeComponents kVirtualMagnitudeVectorZero = {};

      ForceFeedback::SPhysicalActuatorComponents previousPhysicalActuatorValues;
      ForceFeedback::SPhysicalActuatorComponents currentPhysicalActuatorValues;

      const Mapper* mapper = Mapper::GetConfigured(controllerIdentifier);
      bool lastActuationResult = true;

      while (true)
      {
        if (true == lastActuationResult)
          Sleep(kPhysicalForceFeedbackPeriodMilliseconds);
        else
          Sleep(kPhysicalErrorBackoffPeriodMilliseconds);

        if (true == Globals::DoesCurrentProcessHaveInputFocus())
        {
          ForceFeedback::TEffectValue overallEffectGain = 10000;
          ForceFeedback::SPhysicalActuatorComponents physicalActuatorVector = {};
          ForceFeedback::TOrderedMagnitudeComponents virtualMagnitudeVector =
              physicalControllerForceFeedbackBuffer[controllerIdentifier].PlayEffects();

          if (kVirtualMagnitudeVectorZero != virtualMagnitudeVector)
          {
            std::unique_lock lock(physicalControllerForceFeedbackMutex[controllerIdentifier]);

            for (auto virtualController :
                 physicalControllerForceFeedbackRegistration[controllerIdentifier])
              overallEffectGain *=
                  ((ForceFeedback::TEffectValue)virtualController->GetForceFeedbackGain() /
                   ForceFeedback::kEffectModifierMaximum);

            physicalActuatorVector = mapper->MapForceFeedbackVirtualToPhysical(
                virtualMagnitudeVector, overallEffectGain);
          }

          currentPhysicalActuatorValues = physicalActuatorVector;
        }
        else
        {
          currentPhysicalActuatorValues = {};
        }

        if (previousPhysicalActuatorValues != currentPhysicalActuatorValues)
        {
          lastActuationResult =
              WritePhysicalControllerVibration(controllerIdentifier, currentPhysicalActuatorValues);
          previousPhysicalActuatorValues = currentPhysicalActuatorValues;
        }
        else
        {
          lastActuationResult = true;
        }
      }
    }

    static void PollForPhysicalControllerStateChanges(TControllerIdentifier controllerIdentifier)
    {
      SPhysicalState newPhysicalState = physicalControllerState[controllerIdentifier].Get();

      while (true)
      {
        if (EPhysicalDeviceStatus::Ok == newPhysicalState.deviceStatus)
          Sleep(kPhysicalPollingPeriodMilliseconds);
        else
          Sleep(kPhysicalErrorBackoffPeriodMilliseconds);

        newPhysicalState = ReadPhysicalControllerState(controllerIdentifier);

        if (true == physicalControllerState[controllerIdentifier].Update(newPhysicalState))
        {
          const SState newRawVirtualState =
              ((EPhysicalDeviceStatus::Ok == newPhysicalState.deviceStatus)
                   ? Mapper::GetConfigured(controllerIdentifier)
                         ->MapStatePhysicalToVirtual(
                             newPhysicalState,
                             OpaqueControllerSourceIdentifier(controllerIdentifier))
                   : Mapper::GetConfigured(controllerIdentifier)
                         ->MapNeutralPhysicalToVirtual(
                             OpaqueControllerSourceIdentifier(controllerIdentifier)));

          rawVirtualControllerState[controllerIdentifier].Update(newRawVirtualState);
        }
      }
    }

    static void MonitorPhysicalControllerStatus(TControllerIdentifier controllerIdentifier)
    {
      if (controllerIdentifier >= kPhysicalControllerCount)
      {
        Infra::Message::OutputFormatted(
            Infra::Message::ESeverity::Error,
            L"Attempted to monitor physical controller with invalid identifier %u.",
            controllerIdentifier);
        return;
      }

      SPhysicalState oldPhysicalState = GetCurrentPhysicalControllerState(controllerIdentifier);
      SPhysicalState newPhysicalState = oldPhysicalState;

      while (true)
      {
        WaitForPhysicalControllerStateChange(
            controllerIdentifier, newPhysicalState, std::stop_token());

        switch (newPhysicalState.deviceStatus)
        {
          case EPhysicalDeviceStatus::Ok:
            switch (oldPhysicalState.deviceStatus)
            {
              case EPhysicalDeviceStatus::Ok:
                break;
              case EPhysicalDeviceStatus::NotConnected:
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Info,
                    L"Physical controller %u: Hardware connected.",
                    (1 + controllerIdentifier));
                break;
              default:
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Warning,
                    L"Physical controller %u: Cleared previous error condition.",
                    (1 + controllerIdentifier));
                break;
            }
            break;

          case EPhysicalDeviceStatus::NotConnected:
            if (newPhysicalState.deviceStatus != oldPhysicalState.deviceStatus)
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Physical controller %u: Hardware disconnected.",
                  (1 + controllerIdentifier));
            break;

          default:
            if (newPhysicalState.deviceStatus != oldPhysicalState.deviceStatus)
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Warning,
                  L"Physical controller %u: Encountered an error condition.",
                  (1 + controllerIdentifier));
            break;
        }

        oldPhysicalState = newPhysicalState;
      }
    }

    static void Initialize(void)
    {
      static bool isInitialized = false;
      if (true == isInitialized) return;

      static std::once_flag initFlag;
      std::call_once(
          initFlag,
          []() -> void
          {
            // Ensure ALL hHidDevice entries are INVALID_HANDLE_VALUE
            for (auto i = 0; i < kPhysicalControllerCount; ++i)
              hHidDevice[i] = INVALID_HANDLE_VALUE;

            for (auto controllerIdentifier = 0;
                 controllerIdentifier < _countof(physicalControllerState);
                 ++controllerIdentifier)
            {
              const SPhysicalState initialPhysicalState =
                  ReadPhysicalControllerState(controllerIdentifier);
              const SState initialRawVirtualState =
                  Mapper::GetConfigured(controllerIdentifier)
                      ->MapStatePhysicalToVirtual(
                          initialPhysicalState,
                          OpaqueControllerSourceIdentifier(controllerIdentifier));

              physicalControllerState[controllerIdentifier].Set(initialPhysicalState);
              rawVirtualControllerState[controllerIdentifier].Set(initialRawVirtualState);
            }

            TIMECAPS timeCaps;
            MMRESULT timeResult = ImportApiWinMM::timeGetDevCaps(&timeCaps, sizeof(timeCaps));
            if (MMSYSERR_NOERROR == timeResult)
            {
              timeResult = ImportApiWinMM::timeBeginPeriod(timeCaps.wPeriodMin);
              if (MMSYSERR_NOERROR == timeResult)
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Info,
                    L"Set the system timer resolution to %u ms.",
                    timeCaps.wPeriodMin);
              else
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Warning,
                    L"Failed with code %u to set the system timer resolution.",
                    timeResult);
            }
            else
            {
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Warning,
                  L"Failed with code %u to obtain system timer resolution information.",
                  timeResult);
            }

            for (auto controllerIdentifier = 0; controllerIdentifier < kPhysicalControllerCount;
                 ++controllerIdentifier)
            {
              std::thread(PollForPhysicalControllerStateChanges, controllerIdentifier).detach();
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Initialized the physical controller state polling thread for controller %u. Desired polling period is %u ms.",
                  (unsigned int)(1 + controllerIdentifier),
                  kPhysicalPollingPeriodMilliseconds);
            }

            for (auto controllerIdentifier = 0; controllerIdentifier < kPhysicalControllerCount;
                 ++controllerIdentifier)
            {
              hidReadThread[controllerIdentifier] =
                  std::thread(PollHidForShareButton, controllerIdentifier);
              hidReadThread[controllerIdentifier].detach();
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Initialized HID polling thread for controller %u.",
                  (unsigned int)(1 + controllerIdentifier));
            }

            physicalControllerForceFeedbackBuffer =
                new ForceFeedback::Device[kPhysicalControllerCount];
            for (auto controllerIdentifier = 0; controllerIdentifier < kPhysicalControllerCount;
                 ++controllerIdentifier)
            {
              std::thread(ForceFeedbackActuateEffects, controllerIdentifier).detach();
              Infra::Message::OutputFormatted(
                  Infra::Message::ESeverity::Info,
                  L"Initialized the physical controller force feedback actuation thread for controller %u. Desired actuation period is %u ms.",
                  (unsigned int)(1 + controllerIdentifier),
                  kPhysicalForceFeedbackPeriodMilliseconds);
            }

            if (Infra::Message::WillOutputMessageOfSeverity(Infra::Message::ESeverity::Warning))
            {
              for (auto controllerIdentifier = 0; controllerIdentifier < kPhysicalControllerCount;
                   ++controllerIdentifier)
              {
                std::thread(MonitorPhysicalControllerStatus, controllerIdentifier).detach();
                Infra::Message::OutputFormatted(
                    Infra::Message::ESeverity::Info,
                    L"Initialized the physical controller hardware status monitoring thread for controller %u.",
                    (unsigned int)(1 + controllerIdentifier));
              }
            }

            isInitialized = true;
          });
    }

    SCapabilities GetControllerCapabilities(TControllerIdentifier controllerIdentifier)
    {
      Initialize();
      return Mapper::GetConfigured(controllerIdentifier)->GetCapabilities();
    }

    SPhysicalState GetCurrentPhysicalControllerState(TControllerIdentifier controllerIdentifier)
    {
      Initialize();
      return physicalControllerState[controllerIdentifier].Get();
    }

    SState GetCurrentRawVirtualControllerState(TControllerIdentifier controllerIdentifier)
    {
      Initialize();
      return rawVirtualControllerState[controllerIdentifier].Get();
    }

    ForceFeedback::Device* PhysicalControllerForceFeedbackRegister(
        TControllerIdentifier controllerIdentifier, const VirtualController* virtualController)
    {
      Initialize();

      if (controllerIdentifier >= kPhysicalControllerCount)
      {
        Infra::Message::OutputFormatted(
            Infra::Message::ESeverity::Error,
            L"Attempted to register with a physical controller for force feedback with invalid identifier %u.",
            controllerIdentifier);
        return nullptr;
      }

      std::unique_lock lock(physicalControllerForceFeedbackMutex[controllerIdentifier]);
      physicalControllerForceFeedbackRegistration[controllerIdentifier].insert(virtualController);
      return &physicalControllerForceFeedbackBuffer[controllerIdentifier];
    }

    void PhysicalControllerForceFeedbackUnregister(
        TControllerIdentifier controllerIdentifier, const VirtualController* virtualController)
    {
      Initialize();

      if (controllerIdentifier >= kPhysicalControllerCount)
      {
        Infra::Message::OutputFormatted(
            Infra::Message::ESeverity::Error,
            L"Attempted to unregister with a physical controller for force feedback with invalid identifier %u.",
            controllerIdentifier);
        return;
      }

      std::unique_lock lock(physicalControllerForceFeedbackMutex[controllerIdentifier]);
      physicalControllerForceFeedbackRegistration[controllerIdentifier].erase(virtualController);
    }

    bool WaitForPhysicalControllerStateChange(
        TControllerIdentifier controllerIdentifier,
        SPhysicalState& state,
        std::stop_token stopToken)
    {
      Initialize();
      if (controllerIdentifier >= kPhysicalControllerCount) return false;
      return physicalControllerState[controllerIdentifier].WaitForUpdate(state, stopToken);
    }

    bool WaitForRawVirtualControllerStateChange(
        TControllerIdentifier controllerIdentifier, SState& state, std::stop_token stopToken)
    {
      Initialize();
      if (controllerIdentifier >= kPhysicalControllerCount) return false;
      return rawVirtualControllerState[controllerIdentifier].WaitForUpdate(state, stopToken);
    }
  } // namespace Controller
} // namespace Xidi