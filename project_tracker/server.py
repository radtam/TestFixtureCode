"""Local file-backed server for the Cycle Test Project Tracker."""

from __future__ import annotations

import argparse
import copy
import json
import re
import shutil
import threading
import webbrowser
import zipfile
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
DEFAULT_DATA_DIR = APP_DIR / "data"
PROJECT_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9-]{0,63}$")
TASK_STATUSES = {"pending", "in_progress", "blocked", "done"}
PROJECT_STATUSES = {"planning", "active", "on_hold", "complete", "archived"}
MAX_BODY_BYTES = 2_000_000

DEFAULT_TASKS = [
    ("Intake", "Confirm project request and requirements"),
    ("Fixture Setup", "Prepare and inspect the test fixture"),
    ("Test Definition", "Define the cycle sequence and cycle target"),
    ("Calibration/Verification", "Calibrate and verify the load cell"),
    ("Test Run", "Run the planned cycle test"),
    ("Results Review", "Review results against acceptance criteria"),
    ("Closeout", "Document conclusions and close the project"),
]


class ValidationError(ValueError):
    """Raised when project input is invalid."""


class ConflictError(ValueError):
    """Raised when a project ID already exists."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def slugify(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.strip().lower()).strip("-")
    return (slug[:64].rstrip("-") or "project")


def _clean_text(value: Any, maximum: int = 10_000) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise ValidationError("Text fields must contain text.")
    return value.strip()[:maximum]


def _clean_date(value: Any) -> str:
    text = _clean_text(value, 10)
    if text:
        try:
            datetime.strptime(text, "%Y-%m-%d")
        except ValueError as exc:
            raise ValidationError(f"Invalid date: {text}. Use YYYY-MM-DD.") from exc
    return text


def _clean_task(task: Any, index: int) -> dict[str, Any]:
    if not isinstance(task, dict):
        raise ValidationError("Each task must be an object.")
    title = _clean_text(task.get("title"), 200)
    if not title:
        raise ValidationError("Every task needs a title.")
    status = _clean_text(task.get("status") or "pending", 20)
    if status not in TASK_STATUSES:
        raise ValidationError(f"Invalid task status: {status}.")
    task_id = slugify(_clean_text(task.get("id"), 80) or f"task-{index + 1}")
    return {
        "id": task_id,
        "phase": _clean_text(task.get("phase") or "Other", 100),
        "title": title,
        "status": status,
        "owner": _clean_text(task.get("owner"), 100),
        "dueDate": _clean_date(task.get("dueDate")),
        "notes": _clean_text(task.get("notes"), 4_000),
        "blockedReason": _clean_text(task.get("blockedReason"), 1_000),
        "completedAt": _clean_text(task.get("completedAt"), 40) if status == "done" else "",
        "order": index,
    }


def default_tasks() -> list[dict[str, Any]]:
    return [
        {
            "id": f"task-{index + 1}",
            "phase": phase,
            "title": title,
            "status": "pending",
            "owner": "",
            "dueDate": "",
            "notes": "",
            "blockedReason": "",
            "completedAt": "",
            "order": index,
        }
        for index, (phase, title) in enumerate(DEFAULT_TASKS)
    ]


def _clean_history(value: Any) -> list[dict[str, str]]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise ValidationError("Project history must be a list.")
    history = []
    for entry in value[-500:]:
        if not isinstance(entry, dict):
            raise ValidationError("Each history entry must be an object.")
        event = _clean_text(entry.get("event"), 500)
        at = _clean_text(entry.get("at"), 40)
        if event:
            history.append({"at": at or utc_now(), "event": event})
    return history


def calculate_progress(tasks: list[dict[str, Any]]) -> int:
    if not tasks:
        return 0
    weights = {"pending": 0.0, "blocked": 0.0, "in_progress": 0.5, "done": 1.0}
    return round(sum(weights[task["status"]] for task in tasks) / len(tasks) * 100)


def validate_project(payload: Any, existing: dict[str, Any] | None = None) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValidationError("Project data must be a JSON object.")

    name = _clean_text(payload.get("name"), 200)
    if not name:
        raise ValidationError("Project name is required.")
    project_id = slugify(_clean_text(payload.get("id"), 80) or name)
    if not PROJECT_ID_PATTERN.fullmatch(project_id):
        raise ValidationError("Project ID must use lowercase letters, numbers, and hyphens.")

    status = _clean_text(payload.get("status") or "planning", 20)
    if status not in PROJECT_STATUSES:
        raise ValidationError(f"Invalid project status: {status}.")

    raw_tasks = payload.get("tasks")
    if raw_tasks is None:
        raw_tasks = default_tasks()
    if not isinstance(raw_tasks, list):
        raise ValidationError("Tasks must be a list.")
    if len(raw_tasks) > 500:
        raise ValidationError("A project cannot contain more than 500 tasks.")
    tasks = [_clean_task(task, index) for index, task in enumerate(raw_tasks)]
    task_ids = [task["id"] for task in tasks]
    if len(task_ids) != len(set(task_ids)):
        raise ValidationError("Task IDs must be unique.")

    now = utc_now()
    project = {
        "schemaVersion": 1,
        "id": project_id,
        "name": name,
        "status": status,
        "owner": _clean_text(payload.get("owner"), 100),
        "fixtureType": _clean_text(payload.get("fixtureType"), 150),
        "measurementType": _clean_text(payload.get("measurementType"), 50),
        "deviceUnderTest": _clean_text(payload.get("deviceUnderTest"), 250),
        "purpose": _clean_text(payload.get("purpose"), 4_000),
        "requestDate": _clean_date(payload.get("requestDate")),
        "targetDate": _clean_date(payload.get("targetDate")),
        "cycleTarget": max(0, int(payload.get("cycleTarget") or 0)),
        "acceptanceCriteria": _clean_text(payload.get("acceptanceCriteria"), 4_000),
        "risks": _clean_text(payload.get("risks"), 4_000),
        "notes": _clean_text(payload.get("notes"), 10_000),
        "fileLinks": _clean_text(payload.get("fileLinks"), 4_000),
        "tasks": tasks,
        "createdAt": existing.get("createdAt", now) if existing else _clean_text(payload.get("createdAt"), 40) or now,
        "updatedAt": now,
        "archivedAt": existing.get("archivedAt", "") if existing else _clean_text(payload.get("archivedAt"), 40),
        "history": copy.deepcopy(existing.get("history", [])) if existing else _clean_history(payload.get("history")),
    }
    return project


def project_for_response(project: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(project)
    result["progress"] = calculate_progress(result.get("tasks", []))
    return result


def summarize_changes(old: dict[str, Any], new: dict[str, Any]) -> list[str]:
    events: list[str] = []
    if old.get("status") != new.get("status"):
        events.append(f"Project status changed to {new['status'].replace('_', ' ')}")

    old_tasks = {task["id"]: task for task in old.get("tasks", [])}
    new_tasks = {task["id"]: task for task in new.get("tasks", [])}
    for task_id, task in new_tasks.items():
        if task_id not in old_tasks:
            events.append(f"Task added: {task['title']}")
        elif task["status"] != old_tasks[task_id]["status"]:
            events.append(
                f"Task “{task['title']}” changed to {task['status'].replace('_', ' ')}"
            )
    for task_id, task in old_tasks.items():
        if task_id not in new_tasks:
            events.append(f"Task removed: {task['title']}")
    if not events:
        events.append("Project details updated")
    return events


class ProjectStore:
    def __init__(self, data_dir: Path = DEFAULT_DATA_DIR):
        self.data_dir = Path(data_dir)
        self.projects_dir = self.data_dir / "projects"
        self.backups_dir = self.data_dir / "backups"
        self.projects_dir.mkdir(parents=True, exist_ok=True)
        self.backups_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()

    def _path(self, project_id: str) -> Path:
        if not PROJECT_ID_PATTERN.fullmatch(project_id):
            raise ValidationError("Invalid project ID.")
        return self.projects_dir / f"{project_id}.json"

    def _read(self, path: Path) -> dict[str, Any]:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValidationError(f"Could not read {path.name}: {exc}") from exc
        if not isinstance(data, dict):
            raise ValidationError(f"{path.name} does not contain a project object.")
        return data

    def _write(self, project: dict[str, Any]) -> None:
        path = self._path(project["id"])
        temp_path = path.with_suffix(".json.tmp")
        temp_path.write_text(
            json.dumps(project, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temp_path.replace(path)

    def list_projects(self, include_archived: bool = False) -> list[dict[str, Any]]:
        with self._lock:
            projects = []
            for path in sorted(self.projects_dir.glob("*.json")):
                project = self._read(path)
                if include_archived or project.get("status") != "archived":
                    projects.append(project_for_response(project))
            return sorted(projects, key=lambda item: item.get("updatedAt", ""), reverse=True)

    def get(self, project_id: str) -> dict[str, Any]:
        with self._lock:
            path = self._path(project_id)
            if not path.exists():
                raise FileNotFoundError(project_id)
            return project_for_response(self._read(path))

    def create(self, payload: Any) -> dict[str, Any]:
        with self._lock:
            project = validate_project(payload)
            path = self._path(project["id"])
            if path.exists():
                raise ConflictError(f"A project named {project['id']} already exists.")
            event = "Project imported" if project["history"] else "Project created"
            project["history"].append({"at": utc_now(), "event": event})
            self._write(project)
            return project_for_response(project)

    def update(self, project_id: str, payload: Any) -> dict[str, Any]:
        with self._lock:
            path = self._path(project_id)
            if not path.exists():
                raise FileNotFoundError(project_id)
            old = self._read(path)
            project = validate_project(payload, existing=old)
            if project["id"] != project_id:
                raise ValidationError("Project ID cannot be changed after creation.")

            now = project["updatedAt"]
            for task in project["tasks"]:
                old_task = next((item for item in old.get("tasks", []) if item["id"] == task["id"]), None)
                if task["status"] == "done" and not task["completedAt"]:
                    task["completedAt"] = now
                elif old_task and old_task["status"] == "done" and task["status"] != "done":
                    task["completedAt"] = ""
            for event in summarize_changes(old, project):
                project["history"].append({"at": now, "event": event})
            project["history"] = project["history"][-500:]
            self._write(project)
            return project_for_response(project)

    def duplicate(self, project_id: str, requested_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            source = self.get(project_id)
            base = slugify(requested_id or f"{project_id}-copy")
            candidate = base
            number = 2
            while self._path(candidate).exists():
                suffix = f"-{number}"
                candidate = f"{base[:64-len(suffix)].rstrip('-')}{suffix}"
                number += 1
            source["id"] = candidate
            source["name"] = f"{source['name']} (Copy)"
            source["status"] = "planning"
            source["createdAt"] = utc_now()
            source["history"] = []
            source["archivedAt"] = ""
            for task in source["tasks"]:
                task["status"] = "pending"
                task["completedAt"] = ""
            return self.create(source)

    def archive(self, project_id: str) -> dict[str, Any]:
        project = self.get(project_id)
        project["status"] = "archived"
        updated = self.update(project_id, project)
        raw = self._read(self._path(project_id))
        raw["archivedAt"] = raw["updatedAt"]
        self._write(raw)
        return project_for_response(raw)

    def delete(self, project_id: str) -> None:
        with self._lock:
            path = self._path(project_id)
            if not path.exists():
                raise FileNotFoundError(project_id)
            path.unlink()

    def backup(self) -> Path:
        with self._lock:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            path = self.backups_dir / f"project-tracker-backup-{stamp}.zip"
            with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
                for project_path in sorted(self.projects_dir.glob("*.json")):
                    archive.write(project_path, f"projects/{project_path.name}")
            return path

    def restore_backup(self, backup_name: str) -> int:
        if Path(backup_name).name != backup_name or not backup_name.endswith(".zip"):
            raise ValidationError("Invalid backup name.")
        backup_path = self.backups_dir / backup_name
        if not backup_path.exists():
            raise FileNotFoundError(backup_name)
        restored = 0
        with self._lock, zipfile.ZipFile(backup_path) as archive:
            for member in archive.infolist():
                member_path = Path(member.filename)
                if (
                    len(member_path.parts) != 2
                    or member_path.parts[0] != "projects"
                    or member_path.suffix != ".json"
                ):
                    continue
                payload = json.loads(archive.read(member).decode("utf-8"))
                project = validate_project(payload)
                target = self._path(project["id"])
                if target.exists():
                    shutil.copy2(target, target.with_suffix(".json.before-restore"))
                self._write(project)
                restored += 1
        return restored


class TrackerHandler(SimpleHTTPRequestHandler):
    server_version = "ProjectTracker/1.0"

    @property
    def store(self) -> ProjectStore:
        return self.server.store  # type: ignore[attr-defined]

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[{self.log_date_time_string()}] {format % args}")

    def _json(self, status: int, payload: Any) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status: int, message: str) -> None:
        self._json(status, {"error": message})

    def _read_json(self) -> Any:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValidationError("Invalid Content-Length.") from exc
        if length <= 0 or length > MAX_BODY_BYTES:
            raise ValidationError("Request body is empty or too large.")
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValidationError("Request body must be valid JSON.") from exc

    def _parts(self) -> list[str]:
        return [unquote(part) for part in urlparse(self.path).path.split("/") if part]

    def do_GET(self) -> None:
        parts = self._parts()
        if parts[:2] == ["api", "projects"]:
            try:
                if len(parts) == 2:
                    query = urlparse(self.path).query
                    self._json(HTTPStatus.OK, self.store.list_projects("archived=1" in query))
                elif len(parts) == 3:
                    self._json(HTTPStatus.OK, self.store.get(parts[2]))
                else:
                    self._error(HTTPStatus.NOT_FOUND, "Unknown API route.")
            except FileNotFoundError:
                self._error(HTTPStatus.NOT_FOUND, "Project not found.")
            except ValidationError as exc:
                self._error(HTTPStatus.BAD_REQUEST, str(exc))
            return

        if parts[:2] == ["api", "backups"] and len(parts) == 3:
            name = Path(parts[2]).name
            path = self.store.backups_dir / name
            if not path.exists() or name != parts[2] or path.suffix != ".zip":
                self._error(HTTPStatus.NOT_FOUND, "Backup not found.")
                return
            body = path.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/zip")
            self.send_header("Content-Disposition", f'attachment; filename="{name}"')
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if parts and parts[0] == "api":
            self._error(HTTPStatus.NOT_FOUND, "Unknown API route.")
            return

        requested = urlparse(self.path).path
        if requested == "/":
            requested = "/index.html"
        candidate = (STATIC_DIR / requested.lstrip("/")).resolve()
        try:
            candidate.relative_to(STATIC_DIR.resolve())
        except ValueError:
            self._error(HTTPStatus.FORBIDDEN, "Invalid path.")
            return
        if not candidate.is_file():
            candidate = STATIC_DIR / "index.html"
        self.path = "/" + candidate.relative_to(STATIC_DIR).as_posix()
        super().do_GET()

    def do_POST(self) -> None:
        parts = self._parts()
        try:
            if parts == ["api", "projects"]:
                self._json(HTTPStatus.CREATED, self.store.create(self._read_json()))
            elif len(parts) == 4 and parts[:2] == ["api", "projects"] and parts[3] == "duplicate":
                payload = self._read_json()
                requested_id = payload.get("id") if isinstance(payload, dict) else None
                self._json(HTTPStatus.CREATED, self.store.duplicate(parts[2], requested_id))
            elif len(parts) == 4 and parts[:2] == ["api", "projects"] and parts[3] == "archive":
                self._json(HTTPStatus.OK, self.store.archive(parts[2]))
            elif parts == ["api", "backups"]:
                path = self.store.backup()
                self._json(
                    HTTPStatus.CREATED,
                    {"name": path.name, "downloadUrl": f"/api/backups/{path.name}"},
                )
            elif len(parts) == 4 and parts[:2] == ["api", "backups"] and parts[3] == "restore":
                count = self.store.restore_backup(parts[2])
                self._json(HTTPStatus.OK, {"restored": count})
            else:
                self._error(HTTPStatus.NOT_FOUND, "Unknown API route.")
        except ConflictError as exc:
            self._error(HTTPStatus.CONFLICT, str(exc))
        except FileNotFoundError:
            self._error(HTTPStatus.NOT_FOUND, "Project or backup not found.")
        except (ValidationError, ValueError, TypeError, zipfile.BadZipFile) as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))

    def do_PUT(self) -> None:
        parts = self._parts()
        try:
            if len(parts) == 3 and parts[:2] == ["api", "projects"]:
                self._json(HTTPStatus.OK, self.store.update(parts[2], self._read_json()))
            else:
                self._error(HTTPStatus.NOT_FOUND, "Unknown API route.")
        except FileNotFoundError:
            self._error(HTTPStatus.NOT_FOUND, "Project not found.")
        except (ValidationError, ValueError, TypeError) as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))

    def do_DELETE(self) -> None:
        parts = self._parts()
        try:
            if len(parts) == 3 and parts[:2] == ["api", "projects"]:
                self.store.delete(parts[2])
                self._json(HTTPStatus.OK, {"deleted": parts[2]})
            else:
                self._error(HTTPStatus.NOT_FOUND, "Unknown API route.")
        except FileNotFoundError:
            self._error(HTTPStatus.NOT_FOUND, "Project not found.")
        except ValidationError as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))


class TrackerServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], store: ProjectStore):
        super().__init__(address, lambda *args, **kwargs: TrackerHandler(*args, directory=str(STATIC_DIR), **kwargs))
        self.store = store


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the local Cycle Test Project Tracker.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    server = TrackerServer((args.host, args.port), ProjectStore(args.data_dir))
    url = f"http://{args.host}:{server.server_port}"
    print(f"Project Tracker is running at {url}")
    print("Press Ctrl+C in this window to stop it.")
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nProject Tracker stopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
