@echo off
cd /d "%~dp0"
set CIBLE=%APPDATA%\Microsoft\Windows\Start Menu\Programs\Startup\bot_pilote_msfs.bat
if not exist telegram.json (
  echo Lance d'abord lancer_bot.bat une fois pour enregistrer ton bot Telegram.
  pause
  exit /b
)
> "%CIBLE%" echo @echo off
>> "%CIBLE%" echo cd /d "%~dp0"
>> "%CIBLE%" echo start "" pythonw pilote_auto.py
echo Le bot demarrera tout seul, sans fenetre, a chaque ouverture de session Windows.
echo Pour annuler, supprime ce fichier :
echo %CIBLE%
echo.
pause
