# Cycle Test Code Tracker

This is a local tracker for **code work** needed to finish the cycle test machine. Use it to capture each problem, its requirements, and the context you want to hand to AI later. It runs only on this computer, opens in a web browser, and saves readable JSON files in this folder. It does not connect to or change the ESP32 fixture firmware.

## Start the tracker

1. Open the `project_tracker` folder.
2. Double-click `start_tracker.bat`.
3. Keep the black command window open while using the tracker.
4. The tracker should open at `http://127.0.0.1:8765`.

If the browser does not open automatically, enter that address in a browser.

## Stop the tracker

Close the black command window, or click it and press `Ctrl+C`. Projects already saved will not be lost.

## Basic workflow

1. Select **New project**.
2. Fill in the intake fields. Only the project name is required.
   - **Problem**: what is missing or broken
   - **Requirements**: what the finished change must do
   - **Context for AI**: background and details an AI should know
   - **Relevant files / paths**: code or docs to look at
   - **Acceptance criteria**: how you will verify the change
3. When you are ready to work the issue, select **Copy AI brief** and paste it into Cursor or another AI chat.
4. Use the **Tasks** tab only if it helps. New projects start with four optional steps:
   - Confirm requirements
   - Generate code
   - Implement
   - Verify the change works

   Delete any you do not need, or clear them all.
5. Select **Save project**.
6. Update status and task progress as you go.

Progress is calculated from task status when tasks exist. A completed task counts as 100%, an in-progress task counts as 50%, and pending or blocked tasks count as 0%.

## Where data is saved

Each project is stored as a separate file:

`project_tracker\data\projects\<project-id>.json`

These files are normal text files and can be copied with the rest of the repository. Do not edit them while the tracker is running.

## Backups

Select **Create backup** on the dashboard. The tracker:

1. Creates a ZIP file in `project_tracker\data\backups`.
2. Downloads a copy through the browser.

Keep occasional backup ZIP files somewhere outside this repository, such as a backed-up network folder.

## Export and import

- Open a project and select **Export JSON** to download that one project.
- Select **Import JSON** on the dashboard to add an exported project.
- A project ID must be unique. If the ID already exists, duplicate or rename the existing project before importing.

## Archive, duplicate, or delete

Open a saved project and select the `•••` button:

- **Duplicate project** creates a new planning copy and resets its tasks.
- **Archive project** removes it from the normal dashboard. Select **Show archived** to view it.
- **Delete project** asks **Yes** or **No**, then permanently removes the JSON file if you confirm.

Create a backup before deleting important work.

## Troubleshooting

### The tracker does not open

- Confirm the black command window says `Code Tracker is running`.
- Open `http://127.0.0.1:8765` manually.
- If another program uses port 8765, close that program or run:

  `python server.py --port 8766`

  Then open `http://127.0.0.1:8766`.

### Python is not found

This computer had Python available when the tracker was created. If it is later removed, reinstall Python 3 and enable the option to add Python to `PATH`.

### A project will not save

- Make sure the project has a name.
- Project IDs can contain only lowercase letters, numbers, and hyphens.
- Check the message in the lower-right corner of the browser.

### The browser was closed accidentally

If the black command window is still open, return to `http://127.0.0.1:8765`. Saved work remains available. Unsaved edits in the closed browser tab cannot be recovered.

## Technical notes

The app uses only the Python standard library. No package installation or internet connection is required. By default the server listens only on `127.0.0.1`, so other computers cannot access it.
