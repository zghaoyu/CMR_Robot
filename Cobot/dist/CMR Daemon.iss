; 脚本由 Inno Setup 脚本向导 生成！
; 有关创建 Inno Setup 脚本文件的详细资料请查阅帮助文档！

#define MyAppName "CMR Daemon"
#define MyAppVersion "1.0"
#define MyAppPublisher "STO-TECH"
#define MyAppURL "http://www.sto-tech.hk/"
#define MyAppExeName "CMRDaemon.exe"

[Setup]
; 注: AppId的值为单独标识该应用程序。
; 不要为其他安装程序使用相同的AppId值。
; (生成新的GUID，点击 工具|在IDE中生成GUID。)
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
; 注意: 不要在任何共享系统文件上使用“Flags: ignoreversion”

[Icons]
Name: "{commonprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{commondesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

