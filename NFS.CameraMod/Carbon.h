#pragma once

#include <cstdint>   // REQUIRED for uintptr_t

namespace Game
{
	// ------------------------------------------------------------
	// Executable validation
	// ------------------------------------------------------------
	constexpr uintptr_t Entry = 0x0087E926;

	inline const char* Name  =
		"NFSC - Camera Mod";

	inline const char* Error =
		"This .exe is not supported.\n"
		"Please use v1.4 English nfsc.exe (6.88 MB / 7,217,152 bytes).";

	// ------------------------------------------------------------
	// Engine globals
	// ------------------------------------------------------------
	inline float* DeltaTime = reinterpret_cast<float*>(0x00A99A5C);
	inline bool*  IsPaused  = nullptr;

	// ------------------------------------------------------------
	// Functions / addresses
	// ------------------------------------------------------------
	using CreateLookAtFn =
		int(__cdecl*)(void*, void*, void*, void*);

	inline CreateLookAtFn eCreateLookAtMatrix =
		reinterpret_cast<CreateLookAtFn>(0x0071B430);

	// CALL site (not used by MinHook, but kept for reference)
	constexpr uintptr_t HookAddr = 0x00492E5B;

	// Sim::GetTime (ADDRESS, not float!)
	constexpr uintptr_t SimGetTimeAddr = 0x0075CF60;

	// ------------------------------------------------------------
	// Camera internals (v1.4 EN)
	// ------------------------------------------------------------
	constexpr uintptr_t CameraWrapperAddr = 0x00492D80;   // your wrapper start
	constexpr uintptr_t CreateLookAtAddr  = 0x0071B430;
	constexpr uintptr_t DisableTiltsAddr  = 0x00492353;
	constexpr uintptr_t CameraAnchor_Update  = 0x0047968C;
}
