# Cycle Test Project Tracker

This is a local project and task tracker for cycle-test work. It runs only on this computer, opens in a web browser, and saves readable JSON files in this folder. It does not connect to or change the ESP32 fixture firmware.

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
2. Complete the intake fields. Only the project name is required.
3. Review the starter checklist on the **Tasks** tab. Add, remove, reorder, assign, or date tasks as needed.
4. Select **Save project**.
5. Update task status as work proceeds:
   - **Pending**: not started
   - **In progress**: currently being worked
   - **Blocked**: cannot proceed; enter the reason
   - **Done**: completed
6. Use **Timeline** to see dated work and the saved activity history.

Progress is calculated from task status. A completed task counts as 100%, an in-progress task counts as 50%, and pending or blocked tasks count as 0%.

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
- **Delete project** permanently removes its JSON file after requiring confirmation.

Create a backup before deleting important work.

## Troubleshooting

### The tracker does not open

- Confirm the black command window says `Project Tracker is running`.
- Open `http://127.0.0.1:8765` manually.
- If another program uses port 8765, close that program or run:

  `python server.py --port 8766`

  Then open `http://127.0.0.1:8766`.

### Python is not found

This computer had Python available when the tracker was created. If it is later removed, reinstall Python 3 and enable the option to add Python to `PATH`.

### A project will not save

- Make sure the project has a name.
- Make sure every task has a title.
- Project IDs can contain only lowercase letters, numbers, and hyphens.
- Check the message in the lower-right corner of the browser.

### The browser was closed accidentally

If the black command window is still open, return to `http://127.0.0.1:8765`. Saved work remains available. Unsaved edits in the closed browser tab cannot be recovered.

## Technical notes

The app uses only the Python standard library. No package installation or internet connection is required. By default the server listens only on `127.0.0.1`, so other computers cannot access it.
