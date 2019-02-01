// Socket.IO.Server.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "WebSocketServer.h"


int main()
{

	WebSocketServer ws(4503);

	ws.Start();


    return 0;
}

