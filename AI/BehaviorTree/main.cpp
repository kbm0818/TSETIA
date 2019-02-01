#include "behaviac_generated/types/behaviac_types.h"
#include <windows.h>
#include <tchar.h>

#define LOGI printf

static void SetExePath()
{
	TCHAR szCurPath[_MAX_PATH];
	GetModuleFileName(NULL, szCurPath, _MAX_PATH);

	TCHAR* p = szCurPath;

	while (_tcschr(p, L'\\'))
	{
		p = _tcschr(p, L'\\');
		p++;
	}

	*p = L'\0';

	SetCurrentDirectory(szCurPath);
}

FirstAgent* g_FirstAgent = NULL;

bool InitBehavic()
{
	LOGI("InitBehavic\n");

	behaviac::Workspace::GetInstance()->SetFilePath("../tutorials/tutorial_1/cpp/exported");

	behaviac::Workspace::GetInstance()->SetFileFormat(behaviac::Workspace::EFF_xml);

	return true;
}

bool InitPlayer()
{
	LOGI("InitPlayer\n");

	g_FirstAgent = behaviac::Agent::Create<FirstAgent>();

	bool bRet = g_FirstAgent->btload("FirstBT");

	g_FirstAgent->btsetcurrent("FirstBT");

	return bRet;
}

void UpdateLoop()
{
	LOGI("UpdateLoop\n");

	int frames = 0;
	behaviac::EBTStatus status = behaviac::BT_RUNNING;

	while (status == behaviac::BT_RUNNING)
	{
		LOGI("frame %d\n", ++frames);

		status = g_FirstAgent->btexec();
	}
}

void CleanupPlayer()
{
	LOGI("CleanupPlayer\n");

	behaviac::Agent::Destroy(g_FirstAgent);
}

void CleanupBehaviac()
{
	LOGI("CleanupBehaviac\n");

	behaviac::Workspace::GetInstance()->Cleanup();
}


int main()
{
	SetExePath();

	LOGI("BEHAVIAC_CCDEFINE_NAME=%s\n", BEHAVIAC_CCDEFINE_NAME);

	InitBehavic();

	InitPlayer();

	UpdateLoop();

	CleanupPlayer();

	CleanupBehaviac();

	printf("Press any key to continue...");
	int ret = getchar();
	BEHAVIAC_UNUSED_VAR(ret);

	return 0;
}