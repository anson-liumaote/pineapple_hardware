
//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  


//  Copyright (c) 2003-2025 Movella Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

/**********************************************************************

    StackWalker.h


    History:
    2005-07-27   v1    - First public release on http://www.codeproject.com/
    (for additional changes see History in 'StackWalker.cpp'!

 **********************************************************************/
#ifndef STACKWALKER_H
#define STACKWALKER_H

#ifndef __GNUC__

// #pragma once is supported starting with _MCS_VER 1000,
// so we need not to check the version (because we only support _MSC_VER >= 1100)!
//#pragma once

#include <windows.h>

// special defines for VC5/6 (if no actual PSDK is installed):
#if _MSC_VER < 1300
	typedef unsigned __int64 DWORD64, *PDWORD64;
	#if defined(_WIN64)
		typedef unsigned __int64 SIZE_T, *PSIZE_T;
	#else
		typedef unsigned long SIZE_T, *PSIZE_T;
	#endif
#endif  // _MSC_VER < 1300

class StackWalkerInternal;
/*! \brief A class that can help with walking the stack for debugging purposes (Windows version) */
class StackWalker
{
public:
	enum StackWalkOptions
	{
		// No addition info will be retrived
		// (only the address is available)
		RetrieveNone = 0,

		// Try to get the symbol-name
		RetrieveSymbol = 1,

		// Try to get the line for this symbol
		RetrieveLine = 2,

		// Try to retrieve the module-infos
		RetrieveModuleInfo = 4,

		// Also retrieve the version for the DLL/EXE
		RetrieveFileVersion = 8,

		// Contains all the above
		RetrieveVerbose = 0xF,

		// Generate a "good" symbol-search-path
		SymBuildPath = 0x10,

		// Also use the public Microsoft-Symbol-Server
		SymUseSymSrv = 0x20,

		// Contains all the abouve "Sym"-options
		SymAll = 0x30,

		// Contains all options (default)
		OptionsAll = 0x3F
	};

	StackWalker(int options = (int) OptionsAll, // 'int' is by design, to combine the enum-flags
		LPCSTR szSymPath = NULL,
		DWORD dwProcessId = GetCurrentProcessId(),
		HANDLE hProcess = GetCurrentProcess());
	StackWalker(DWORD dwProcessId, HANDLE hProcess);
	virtual ~StackWalker();

	typedef BOOL (__stdcall* PReadProcessMemoryRoutine)(
		HANDLE      hProcess,
		DWORD64     qwBaseAddress,
		PVOID       lpBuffer,
		DWORD       nSize,
		LPDWORD     lpNumberOfBytesRead,
		LPVOID      pUserData  // optional data, which was passed in "ShowCallstack"
	);

	BOOL LoadModules();

	BOOL ShowCallstack(HANDLE hThread = GetCurrentThread(),
		const CONTEXT* context = NULL,
		PReadProcessMemoryRoutine readMemoryFunction = NULL,
		LPVOID pUserData = NULL  // optional to identify some data in the 'readMemoryFunction'-callback
	);

protected:
	enum { STACKWALK_MAX_NAMELEN = 1024 }; // max name length for found symbols

	/*! \brief Container for each Callstack-Entry */
	struct CallstackEntry
	{
		DWORD64 offset;  // if 0, we have no valid entry
		CHAR name[STACKWALK_MAX_NAMELEN];
		CHAR undName[STACKWALK_MAX_NAMELEN];
		CHAR undFullName[STACKWALK_MAX_NAMELEN];
		DWORD64 offsetFromSmybol;
		DWORD offsetFromLine;
		DWORD lineNumber;
		CHAR lineFileName[STACKWALK_MAX_NAMELEN];
		DWORD symType;
		LPCSTR symTypeString;
		CHAR moduleName[STACKWALK_MAX_NAMELEN];
		DWORD64 baseOfImage;
		CHAR loadedImageName[STACKWALK_MAX_NAMELEN];
	};

	enum CallstackEntryType {firstEntry, nextEntry, lastEntry};

	virtual void OnSymInit(LPCSTR szSearchPath, DWORD symOptions, LPCSTR szUserName);
	virtual void OnLoadModule(LPCSTR img, LPCSTR mod, DWORD64 baseAddr, DWORD size, DWORD result, LPCSTR symType, LPCSTR pdbName, ULONGLONG fileVersion);
	virtual void OnCallstackEntry(CallstackEntryType eType, CallstackEntry& entry);
	virtual void OnDbgHelpErr(LPCSTR szFuncName, DWORD gle, DWORD64 addr);
	virtual void OnOutput(LPCSTR szText);

	StackWalkerInternal* m_sw;
	HANDLE m_hProcess;
	DWORD m_dwProcessId;
	BOOL m_modulesLoaded;
	LPSTR m_szSymPath;

	int m_options;

	static BOOL __stdcall myReadProcMem(HANDLE hProcess, DWORD64 qwBaseAddress, PVOID lpBuffer, DWORD nSize, LPDWORD lpNumberOfBytesRead);

	friend StackWalkerInternal;
	StackWalker(StackWalker const&) = delete;
	StackWalker& operator = (StackWalker const&) = delete;
};

// The "ugly" assembler-implementation is needed for systems before XP
// If you have a new PSDK and you only compile for XP and later, then you can use
// the "RtlCaptureContext"
// Currently there is no define which determines the PSDK-Version...
// So we just use the compiler-version (and assumes that the PSDK is
// the one which was installed by the VS-IDE)

// INFO: If you want, you can use the RtlCaptureContext if you only target XP and later...
//       But I currently use it in x64/IA64 environments...
//#if defined(_M_IX86) && (_WIN32_WINNT <= 0x0500) && (_MSC_VER < 1400)

#if defined(_M_IX86)
#ifdef CURRENT_THREAD_VIA_EXCEPTION
// The following is not a "perfect" implementation,
// because the callstack is only valid in the "__except" block...
#define GET_CURRENT_CONTEXT(c, contextFlags) \
	do { \
		memset(&c, 0, sizeof(CONTEXT)); \
		EXCEPTION_POINTERS *pExp = NULL; \
		__try { \
			throw 0; \
		} __except( ( (pExp = GetExceptionInformation()) ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_EXECUTE_HANDLER)) {} \
		if (pExp != NULL) \
			memcpy(&c, pExp->ContextRecord, sizeof(CONTEXT)); \
		c.ContextFlags = contextFlags; \
	} while(0)
#else
// The following should be enough for walking the callstack...
#define GET_CURRENT_CONTEXT(c, contextFlags) \
	do { \
		memset(&c, 0, sizeof(CONTEXT)); \
		c.ContextFlags = contextFlags; \
		__asm    call x \
		__asm x: pop eax \
		__asm    mov c.Eip, eax \
		__asm    mov c.Ebp, ebp \
		__asm    mov c.Esp, esp \
	} while(0)
#endif

#else

// The following is defined for x86 (XP and higher), x64 and IA64:
#define GET_CURRENT_CONTEXT(c, contextFlags) \
	do { \
		memset(&c, 0, sizeof(CONTEXT)); \
		c.ContextFlags = contextFlags; \
		RtlCaptureContext(&c); \
	} while(0)
#endif

#endif

#endif
