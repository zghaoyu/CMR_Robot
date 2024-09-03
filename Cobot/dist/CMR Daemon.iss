; �ű��� Inno Setup �ű��� ���ɣ�
; �йش��� Inno Setup �ű��ļ�����ϸ��������İ����ĵ���

#define MyAppName "CMR Daemon"
#define MyAppVersion "1.0"
#define MyAppPublisher "STO-TECH"
#define MyAppURL "http://www.sto-tech.hk/"
#define MyAppExeName "CMRDaemon.exe"

[Setup]
; ע: AppId��ֵΪ������ʶ��Ӧ�ó���
; ��ҪΪ������װ����ʹ����ͬ��AppIdֵ��
; (�����µ�GUID����� ����|��IDE������GUID��)
AppId={{4FE876F3-0FA8-4B9F-89C9-C695CCC6BCD1}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={pf}\{#MyAppName}
DisableProgramGroupPage=yes
LicenseFile=E:\MyDoc\Workspace_Python\Cobot\dist\description_64.txt
InfoBeforeFile=E:\MyDoc\Workspace_Python\Cobot\dist\before installation.txt
InfoAfterFile=E:\MyDoc\Workspace_Python\Cobot\dist\after installation.txt
OutputBaseFilename=CMR Daemon Setup
Compression=lzma
SolidCompression=yes

[Languages]
Name: "chinesesimp"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 0,6.1

[Files]
Source: "E:\MyDoc\Workspace_Python\Cobot\dist\CMRDaemon.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\MyDoc\Workspace_Python\Cobot\dist\CMR.ico"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\MyDoc\Workspace_Python\Cobot\dist\libiconv.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\MyDoc\Workspace_Python\Cobot\dist\libzbar-64.dll"; DestDir: "{app}"; Flags: ignoreversion
; ע��: ��Ҫ���κι���ϵͳ�ļ���ʹ�á�Flags: ignoreversion��

[Icons]
Name: "{commonprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{commondesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

