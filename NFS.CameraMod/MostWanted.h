#pragma once

#include <cstdint>   // REQUIRED for uintptr_t

namespace Game
{
	// ------------------------------------------------------------
	// Executable validation
	// ------------------------------------------------------------
	constexpr uintptr_t Entry = 0x007C4040;

	inline const char* Name  =
		"NFSMW - Camera Mod";

	inline const char* Error =
		"This .exe is not supported.\n"
		"Please use v1.3 Reloaded speed.exe (5,75 MB (6.029.312 bytes).";

	// ------------------------------------------------------------
	// Engine globals
	// ------------------------------------------------------------
	inline float* DeltaTime = reinterpret_cast<float*>(0x009259BC);
	inline bool*  IsPaused  = nullptr;

	// ------------------------------------------------------------
	// Functions / addresses
	// ------------------------------------------------------------
	using CreateLookAtFn =
		int(__cdecl*)(void*, void*, void*, void*);
	
	inline CreateLookAtFn eCreateLookAtMatrix =
		reinterpret_cast<CreateLookAtFn>(0x006CF0A0);

	// CALL site (not used by MinHook, but kept for reference)
	constexpr uintptr_t HookAddr = 0X0047DCBC;

	// Sim::GetTime (ADDRESS, not float!)
	constexpr uintptr_t SimGetTimeAddr = 0x006E8DE0;

	// ------------------------------------------------------------
	// Camera internals (v1.3 EN)
	// ------------------------------------------------------------
	constexpr uintptr_t CameraWrapperAddr = 0x0047DBF6;   // your wrapper start
	constexpr uintptr_t CreateLookAtAddr  = 0x006CF0A0;
	constexpr uintptr_t DisableTiltsAddr  = 0x0047D3C0;
	
}