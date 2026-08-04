import json
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import zipfile
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from server import (  # noqa: E402
    ConflictError,
    ProjectStore,
    TrackerServer,
    ValidationError,
    calculate_progress,
    migrate_project,
    validate_project,
)


def sample_project(project_id: str = "current-cycle-feature") -> dict:
    return {
        "id": project_id,
        "name": "Persist current cycle count",
        "status": "planning",
        "owner": "Test Engineering",
        "problem": "The fixture does not remember the cycle it just finished.",
        "requirements": "Store current cycle on the ESP32 and expose it to the display.",
        "context": "This is for one active test at a time.",
        "relevantFiles": "TestFixtureV5.5/TestFixtureV7.ino",
        "acceptanceCriteria": "After a power cycle mid-run, the display can recover the last completed cycle.",
        "constraints": "Do not break existing LIVE_TEST start/pause/stop behavior.",
        "targetDate": "2026-08-20",
        "tasks": [
            {
                "id": "confirm-requirements",
                "phase": "Requirements",
                "title": "Confirm requirements",
                "status": "pending",
                "owner": "",
                "dueDate": "2026-08-05",
                "notes": "",
                "blockedReason": "",
            },
            {
                "id": "implement",
                "phase": "Implementation",
                "title": "Implement",
                "status": "in_progress",
                "owner": "",
                "dueDate": "2026-08-15",
                "notes": "",
                "blockedReason": "",
            },
        ],
    }


class ProjectValidationTests(unittest.TestCase):
    def test_required_name_and_valid_dates(self):
        with self.assertRaisesRegex(ValidationError, "name is required"):
            validate_project({})
        invalid = sample_project()
        invalid["targetDate"] = "08/20/2026"
        with self.assertRaisesRegex(ValidationError, "Invalid date"):
            validate_project(invalid)

    def test_invalid_task_is_rejected(self):
        invalid = sample_project()
        invalid["tasks"][0]["status"] = "almost-done"
        with self.assertRaisesRegex(ValidationError, "Invalid task status"):
            validate_project(invalid)

    def test_progress_weights_in_progress_as_half(self):
        project = validate_project(sample_project())
        self.assertEqual(calculate_progress(project["tasks"]), 25)

    def test_default_tasks_are_code_workflow(self):
        project = validate_project({"name": "Blank starter"})
        titles = [task["title"] for task in project["tasks"]]
        self.assertEqual(
            titles,
            [
                "Confirm requirements",
                "Generate code",
                "Implement",
                "Verify the change works",
            ],
        )

    def test_migrate_old_schema(self):
        old = {
            "schemaVersion": 1,
            "id": "legacy",
            "name": "Legacy",
            "purpose": "Old purpose text",
            "fileLinks": "a.ino",
            "risks": "Do not break pause",
            "fixtureType": "Torque fixture",
            "tasks": [],
        }
        migrated = migrate_project(old)
        self.assertEqual(migrated["schemaVersion"], 2)
        self.assertEqual(migrated["problem"], "Old purpose text")
        self.assertEqual(migrated["relevantFiles"], "a.ino")
        self.assertEqual(migrated["constraints"], "Do not break pause")
        self.assertEqual(migrated["hardwareNotes"], "Torque fixture")


class ProjectStoreTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.store = ProjectStore(Path(self.temporary.name))

    def tearDown(self):
        self.temporary.cleanup()

    def test_create_load_and_duplicate_conflict(self):
        created = self.store.create(sample_project())
        self.assertEqual(created["id"], "current-cycle-feature")
        self.assertEqual(created["progress"], 25)
        self.assertEqual(created["problem"], "The fixture does not remember the cycle it just finished.")
        self.assertEqual(self.store.get("current-cycle-feature")["name"], "Persist current cycle count")
        with self.assertRaises(ConflictError):
            self.store.create(sample_project())

    def test_update_records_task_completion_and_history(self):
        project = self.store.create(sample_project())
        project["status"] = "active"
        project["tasks"][0]["status"] = "done"
        updated = self.store.update(project["id"], project)
        self.assertTrue(updated["tasks"][0]["completedAt"])
        events = [entry["event"] for entry in updated["history"]]
        self.assertIn("Project status changed to active", events)
        self.assertTrue(any("Confirm requirements" in event and "done" in event for event in events))

    def test_duplicate_resets_tasks_and_uses_unique_id(self):
        self.store.create(sample_project())
        copy = self.store.duplicate("current-cycle-feature")
        second_copy = self.store.duplicate("current-cycle-feature")
        self.assertEqual(copy["id"], "current-cycle-feature-copy")
        self.assertEqual(second_copy["id"], "current-cycle-feature-copy-2")
        self.assertTrue(all(task["status"] == "pending" for task in copy["tasks"]))

    def test_archive_filter_and_delete(self):
        self.store.create(sample_project())
        archived = self.store.archive("current-cycle-feature")
        self.assertEqual(archived["status"], "archived")
        self.assertEqual(self.store.list_projects(), [])
        self.assertEqual(len(self.store.list_projects(include_archived=True)), 1)
        self.store.delete("current-cycle-feature")
        with self.assertRaises(FileNotFoundError):
            self.store.get("current-cycle-feature")

    def test_backup_contains_projects_and_can_restore(self):
        self.store.create(sample_project())
        backup = self.store.backup()
        self.assertTrue(backup.exists())
        with zipfile.ZipFile(backup) as archive:
            self.assertIn("projects/current-cycle-feature.json", archive.namelist())
        self.store.delete("current-cycle-feature")
        self.assertEqual(self.store.restore_backup(backup.name), 1)
        restored = self.store.get("current-cycle-feature")
        self.assertEqual(restored["requirements"], "Store current cycle on the ESP32 and expose it to the display.")
        self.assertEqual(restored["history"][0]["event"], "Project created")

    def test_import_preserves_existing_history(self):
        payload = sample_project("imported-test")
        payload["history"] = [{"at": "2026-07-01T12:00:00Z", "event": "Original intake completed"}]
        imported = self.store.create(payload)
        self.assertEqual(imported["history"][0]["event"], "Original intake completed")
        self.assertEqual(imported["history"][-1]["event"], "Project imported")


class HttpSmokeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.server = TrackerServer(("127.0.0.1", 0), ProjectStore(Path(self.temporary.name)))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.temporary.cleanup()

    def request(self, path: str, method: str = "GET", payload=None):
        body = json.dumps(payload).encode() if payload is not None else None
        request = urllib.request.Request(
            self.base_url + path,
            data=body,
            method=method,
            headers={"Content-Type": "application/json"} if body else {},
        )
        with urllib.request.urlopen(request, timeout=2) as response:
            return response.status, response.headers.get_content_type(), response.read()

    def test_static_page_and_project_api_persist(self):
        status, content_type, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertEqual(content_type, "text/html")
        self.assertIn(b"Code Work Tracker", body)

        status, _, body = self.request("/api/projects", "POST", sample_project("api-test"))
        self.assertEqual(status, 201)
        created = json.loads(body)
        self.assertEqual(created["id"], "api-test")
        self.assertEqual(created["schemaVersion"], 2)

        status, _, body = self.request("/api/projects/api-test")
        self.assertEqual(status, 200)
        loaded = json.loads(body)
        self.assertEqual(loaded["problem"], "The fixture does not remember the cycle it just finished.")
        self.assertIn("relevantFiles", loaded)

    def test_malformed_json_returns_400(self):
        request = urllib.request.Request(
            self.base_url + "/api/projects",
            data=b"{not valid",
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with self.assertRaises(urllib.error.HTTPError) as caught:
            urllib.request.urlopen(request, timeout=2)
        error = caught.exception
        self.assertEqual(error.code, 400)
        self.assertIn("valid JSON", error.read().decode())
        error.close()


if __name__ == "__main__":
    unittest.main()
