# Lab0x07 terminal commands

Run the following commands in Windows PowerShell.

## Push changes to GitHub

```powershell
cd "C:\Users\jorda\Documents\Romi Robot\Project Design\ME-4305\Lab0x07"
git status
git add -- .
git commit -m "Update Lab0x07"
git push origin main
```

Review `git status` before staging. These commands stage this folder's changes, save a commit, and upload it to GitHub.

## Pull changes from GitHub

```powershell
cd "C:\Users\jorda\Documents\Romi Robot\Project Design\ME-4305\Lab0x07"
git status
git pull --ff-only
```

This downloads updates for the entire repository and advances the current branch without creating a merge commit. Commit or stash local edits first if they conflict with incoming changes.

## Connect to the MicroPython REPL

The REPL (Read-Evaluate-Print Loop) lets you execute Python commands directly on the Nucleo. Connect through the Shoe of Brian USB port and close PuTTY or any other application using the serial port.

Install the connection tool once:

```powershell
py -m pip install mpremote
```

List ports and connect, replacing `COM5` with the board's port:

```powershell
mpremote connect list
mpremote connect COM5 repl
```

Press **Ctrl+C** to interrupt a running program and reach the `>>>` prompt. To run the copy of `main.py` already saved on the board, enter this at `>>>`:

```python
execfile("main.py")
```
# to upload pc files to the board
mpremote connect COM5 fs cp main.py :main.py
mpremote connect COM5 fs cp taskmotor.py :taskmotor.py

# to run the uploaded application
mpremote connect COM5 exec "execfile('main.py')"
mpremote connect COM5 exec "execfile('main.py')" repl

This runs the board's application, including startup calibration and the task scheduler. Required modules and calibration files must also be on the board. It does not upload your PC's current files.

Press **Ctrl+C** to interrupt execution. Press **Ctrl+D** at the prompt to soft-reset MicroPython; the board's `main.py` runs automatically at startup. Press **Ctrl+]** to exit the mpremote terminal.
