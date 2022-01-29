# Team 3647 2022 Robot Code

## Configuring Auto formatting

### VSCode Extensions
Install the following extensions:

- `vscjava.vscode-gradle`
- `richardwillis.vscode-spotless-gradle`

### After cloning the repository:
Then select a `.java` file and type `Ctrl+Shift+P`:
1. Select "Format document with"
2. Select "Config Default formatter"
3. Select "Spotless Gradle"

### FOR WINDOWS ONLY (if you are getting the line endings error for spotless):
* go to the bottom right corner of your vscode, you should see CRLF as your line ending type
* click on it and change the "End of Line Sequence* to LF
* this works on a per file basis, meaning you must perform these steps for every file you create on windows

## Characterization
Use `SysId` tool from WPILIB, place all good configuration files in the corresponding directories
- `config/sysid/<subsystem>/<ISO-date>-config-<description>.json`
- ISO date is `YYYY_MM_DD` so Jan 17th 2022 is `2022_01_17`
- This way the configs will be ordered by date in the file manager

## Keeping track of vision pipelines
Save all good pipelines to `config/vision/<ISO-date>_description`