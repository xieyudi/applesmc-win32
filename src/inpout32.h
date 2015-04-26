#pragma once

//Functions exported from DLL.
//For easy inclusion is user projects.
void	__stdcall Out32(short PortAddress, short data);
short	__stdcall Inp32(short PortAddress);