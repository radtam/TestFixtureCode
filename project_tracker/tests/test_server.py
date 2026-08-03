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
    validate_project,
)


def sample_project(project_id: str = "latch-test") -> dict:
    return {
        "id": project_id,
        "name": "Latch endurance test",
        "status": "planning",
        "owner": "Test Engineering",
        "fixtureType": "Force fixture",
        "measurementType": "Force",
        "targetDate": "2026-08-20",
        "cycleTarget": 1000,
        "tasks": [
            {
                "id": "fixture-check",
                "phase": "Fixture Setup",
                "title": "Inspect fixture",
                "status": "pending",
                "owner": "",
                "dueDate": "2026-08-05",
                "notes": "",
                "blockedReason": "",
            },
            {
                "id": "run-test",
                "phase": "Test Run",
                "title": "Run 1,000 cycles",
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


class ProjectStoreTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.store = ProjectStore(Path(self.temporary.name))

    def tearDown(self):
        self.temporary.cleanup()

    def test_create_load_and_duplicate_conflict(self):
        created = self.store.create(sample_project())
        self.assertEqual(created["id"], "latch-test")
        self.assertEqual(created["progress"], 25)
        self.assertEqual(self.store.get("latch-test")["name"], "Latch endurance test")
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
        self.assertTrue(any("Inspect fixture" in event and "done" in event for event in events))

    def test_duplicate_resets_tasks_and_uses_unique_id(self):
        self.store.create(sample_project())
        copy = self.store.duplicate("latch-test")
        second_copy = self.store.duplicate("latch-test")
        self.assertEqual(copy["id"], "latch-test-copy")
        self.assertEqual(second_copy["id"], "latch-test-copy-2")
        self.assertTrue(all(task["status"] == "pending" for task in copy["tasks"]))

    def test_archive_filter_and_delete(self):
        self.store.create(sample_project())
        archived = self.store.archive("latch-test")
        self.assertEqual(archived["status"], "archived")
        self.assertEqual(self.store.list_projects(), [])
        self.assertEqual(len(self.store.list_projects(include_archived=True)), 1)
        self.store.delete("latch-test")
        with self.assertRaises(FileNotFoundError):
            self.store.get("latch-test")

    def test_backup_contains_projects_and_can_restore(self):
        self.store.create(sample_project())
        backup = self.store.backup()
        self.assertTrue(backup.exists())
        with zipfile.ZipFile(backup) as archive:
            self.assertIn("projects/latch-test.json", archive.namelist())
        self.store.delete("latch-test")
        self.assertEqual(self.store.restore_backup(backup.name), 1)
        restored = self.store.get("latch-test")
        self.assertEqual(restored["cycleTarget"], 1000)
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
        self.assertIn(b"Cycle Test Project Tracker", body)

        status, _, body = self.request("/api/projects", "POST", sample_project("api-test"))
        self.assertEqual(status, 201)
        self.assertEqual(json.loads(body)["id"], "api-test")

        status, _, body = self.request("/api/projects/api-test")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["cycleTarget"], 1000)

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
