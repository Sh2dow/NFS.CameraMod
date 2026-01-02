#include <windows.h>
#include "Game.h"
// ==========================================================
// DLL ENTRY
// ==========================================================
BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(hModule);

        uintptr_t base = (uintptr_t)GetModuleHandleA(NULL);
        auto* dos = (IMAGE_DOS_HEADER*)base;
        auto* nt = (IMAGE_NT_HEADERS*)(base + dos->e_lfanew);

        if (base + nt->OptionalHeader.AddressOfEntryPoint + (0x400000 - base) == Game::Entry)
        {
            if (InstallHooks())
                OutputDebugStringA("[CameraMod] Hooks installed.\n");
        }
        else
        {
            MessageBoxA(NULL, Game::Error, Game::Name, MB_ICONERROR);
            return FALSE;
        }
    }
    return TRUE;
}
