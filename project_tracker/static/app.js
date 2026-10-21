"use strict";

const PHASE_TASKS = [
  ["Requirements", "Confirm requirements"],
  ["Design", "Generate code"],
  ["Implementation", "Implement"],
  ["Verification", "Verify the change works"],
];

const FORM_FIELDS = [
  "id", "name", "status", "owner", "requestDate", "targetDate",
  "problem", "requirements", "context", "relevantFiles",
  "acceptanceCriteria", "constraints", "hardwareNotes", "notes",
];

const state = {
  projects: [],
  current: null,
  tasks: [],
  isNew: false,
  dirty: false,
  activeTab: "overview",
  confirmResolve: null,
};

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];

function makeDefaultTasks() {
  return PHASE_TASKS.map(([phase, title], index) => ({
    id: `task-${index + 1}`,
    phase,
    title,
    status: "pending",
    owner: "",
    dueDate: "",
    notes: "",
    blockedReason: "",
    completedAt: "",
    order: index,
  }));
}

async function api(path, options = {}) {
  const config = { ...options, headers: { ...(options.headers || {}) } };
  if (config.body && typeof config.body !== "string") {
    config.headers["Content-Type"] = "application/json";
    config.body = JSON.stringify(config.body);
  }
  const response = await fetch(path, config);
  const contentType = response.headers.get("content-type") || "";
  const result = contentType.includes("application/json") ? await response.json() : null;
  if (!response.ok) throw new Error(result?.error || `Request failed (${response.status})`);
  return result;
}

function showToast(message, isError = false) {
  const toast = $("#toast");
  toast.textContent = message;
  toast.classList.toggle("error", isError);
  toast.hidden = false;
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => { toast.hidden = true; }, 3600);
}

function askConfirm(message, title = "Are you sure?") {
  return new Promise((resolve) => {
    state.confirmResolve = resolve;
    $("#confirmTitle").textContent = title;
    $("#confirmMessage").textContent = message;
    $("#confirmModal").hidden = false;
    $("#confirmYes").focus();
  });
}

function closeConfirm(answer) {
  $("#confirmModal").hidden = true;
  const resolve = state.confirmResolve;
  state.confirmResolve = null;
  if (resolve) resolve(answer);
}

function displayStatus(value) {
  return (value || "planning").replaceAll("_", " ").replace(/\b\w/g, (letter) => letter.toUpperCase());
}

function formatDate(value, fallback = "No date") {
  if (!value) return fallback;
  const date = new Date(`${value}T12:00:00`);
  return Number.isNaN(date.getTime())
    ? value
    : new Intl.DateTimeFormat(undefined, { month: "short", day: "numeric", year: "numeric" }).format(date);
}

function formatTimestamp(value) {
  if (!value) return "";
  const date = new Date(value);
  return Number.isNaN(date.getTime())
    ? value
    : new Intl.DateTimeFormat(undefined, { dateStyle: "medium", timeStyle: "short" }).format(date);
}

function isOverdue(task) {
  if (!task.dueDate || task.status === "done") return false;
  const today = new Date();
  today.setHours(0, 0, 0, 0);
  return new Date(`${task.dueDate}T00:00:00`) < today;
}

function taskProgress(tasks) {
  if (!tasks.length) return 0;
  const weights = { pending: 0, blocked: 0, in_progress: 0.5, done: 1 };
  return Math.round(tasks.reduce((sum, task) => sum + (weights[task.status] || 0), 0) / tasks.length * 100);
}

function makeElement(tag, className, text) {
  const element = document.createElement(tag);
  if (className) element.className = className;
  if (text !== undefined) element.textContent = text;
  return element;
}

function previewText(project) {
  return project.problem || project.requirements || project.context || "";
}

async function loadProjects() {
  try {
    const suffix = $("#showArchived").checked ? "?archived=1" : "";
    state.projects = await api(`/api/projects${suffix}`);
    renderDashboard();
  } catch (error) {
    showToast(error.message, true);
  }
}

function renderDashboard() {
  const query = $("#searchInput").value.trim().toLowerCase();
  const visible = state.projects.filter((project) =>
    [project.name, project.id, project.owner, project.problem, project.requirements, project.hardwareNotes]
      .some((value) => String(value || "").toLowerCase().includes(query))
  );
  const active = state.projects.filter((project) => ["planning", "active", "on_hold"].includes(project.status));
  const allTasks = state.projects.flatMap((project) => project.tasks || []);
  $("#activeCount").textContent = active.length;
  $("#completedTaskCount").textContent = allTasks.filter((task) => task.status === "done").length;
  $("#overdueCount").textContent = allTasks.filter(isOverdue).length;
  $("#averageProgress").textContent = `${state.projects.length
    ? Math.round(state.projects.reduce((sum, project) => sum + project.progress, 0) / state.projects.length)
    : 0}%`;

  const grid = $("#projectGrid");
  grid.replaceChildren();
  visible.forEach((project) => grid.append(createProjectCard(project)));
  $("#emptyState").hidden = state.projects.length !== 0;
  if (state.projects.length && !visible.length) {
    grid.append(makeElement("p", "muted-message", "No projects match your search."));
  }
}

function createProjectCard(project) {
  const card = makeElement("article", "project-card");
  card.tabIndex = 0;
  card.setAttribute("role", "button");
  card.setAttribute("aria-label", `Open ${project.name}`);
  card.addEventListener("click", () => openProject(project.id));
  card.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") openProject(project.id);
  });

  const status = makeElement("span", `status-badge ${project.status}`, displayStatus(project.status));
  const name = makeElement("h3", "", project.name);
  const meta = makeElement("p", "project-meta");
  const summary = previewText(project);
  const metaParts = [
    summary ? summary.slice(0, 120) + (summary.length > 120 ? "…" : "") : "",
    project.owner && `Owner: ${project.owner}`,
  ].filter(Boolean);
  meta.textContent = metaParts.join(" · ") || "No problem or requirements entered yet";

  const footer = makeElement("div", "project-card-footer");
  const progressHeader = makeElement("div", "card-progress-header");
  progressHeader.append(makeElement("span", "", "Progress"), makeElement("strong", "", `${project.progress}%`));
  const track = makeElement("div", "progress-track");
  const fill = makeElement("div", "progress-fill");
  fill.style.width = `${project.progress}%`;
  track.append(fill);

  const due = makeElement("div", "card-due");
  const overdueTasks = (project.tasks || []).filter(isOverdue).length;
  const today = new Date();
  today.setHours(0, 0, 0, 0);
  const targetOverdue = project.targetDate
    && new Date(`${project.targetDate}T00:00:00`) < today
    && project.status !== "complete";
  const target = makeElement(
    "span",
    targetOverdue ? "overdue" : "",
    project.targetDate ? `Target ${formatDate(project.targetDate)}` : "No target date"
  );
  due.append(target, makeElement("span", overdueTasks ? "overdue" : "", overdueTasks ? `${overdueTasks} overdue` : ""));
  footer.append(progressHeader, track, due);
  card.append(status, name, meta, footer);
  return card;
}

function showEditor() {
  $("#dashboardView").hidden = true;
  $("#editorView").hidden = false;
  window.scrollTo({ top: 0, behavior: "instant" });
}

async function showDashboard() {
  if (state.dirty) {
    const leave = await askConfirm("You have unsaved changes. Leave without saving?", "Leave project?");
    if (!leave) return;
  }
  state.current = null;
  state.dirty = false;
  $("#editorView").hidden = true;
  $("#dashboardView").hidden = false;
  $("#actionMenu").hidden = true;
  loadProjects();
}

function newProject() {
  state.current = null;
  state.tasks = makeDefaultTasks();
  state.isNew = true;
  state.dirty = false;
  $("#projectForm").reset();
  $("#projectForm").elements.status.value = "planning";
  $("#projectForm").elements.requestDate.value = new Date().toISOString().slice(0, 10);
  $("#projectForm").elements.id.readOnly = false;
  $("#editorProjectId").textContent = "NEW PROJECT";
  $("#editorTitle").textContent = "Project intake";
  $("#archiveButton").textContent = "Archive project";
  $("#archiveButton").disabled = false;
  $("#actionMenu").hidden = true;
  renderEditor();
  switchTab("overview");
  showEditor();
  $("#projectForm").elements.name.focus();
}

async function openProject(projectId) {
  try {
    const project = await api(`/api/projects/${encodeURIComponent(projectId)}`);
    state.current = project;
    state.tasks = structuredClone(project.tasks || []);
    state.isNew = false;
    state.dirty = false;
    populateForm(project);
    $("#editorProjectId").textContent = project.id;
    $("#editorTitle").textContent = project.name;
    $("#archiveButton").textContent = project.status === "archived" ? "Project archived" : "Archive project";
    $("#archiveButton").disabled = project.status === "archived";
    renderEditor();
    switchTab("overview");
    showEditor();
  } catch (error) {
    showToast(error.message, true);
  }
}

function populateForm(project) {
  const form = $("#projectForm");
  FORM_FIELDS.forEach((field) => {
    if (form.elements[field]) form.elements[field].value = project[field] ?? "";
  });
  form.elements.id.readOnly = true;
}

function renderEditor() {
  renderTasks();
  renderProgress();
  renderTimeline();
  $("#saveState").textContent = state.dirty ? "Unsaved changes" : state.isNew ? "Not saved" : "All changes saved";
  $("#taskTabCount").textContent = state.tasks.length;
  $("#moreButton").hidden = state.isNew;
  $("#exportButton").disabled = state.isNew;
}

function renderProgress() {
  const progress = taskProgress(state.tasks);
  $("#editorProgress").textContent = `${progress}%`;
  $("#progressRing").style.setProperty("--progress", `${progress}%`);
  const phases = new Map();
  state.tasks.forEach((task) => {
    const phase = task.phase || "Other";
    if (!phases.has(phase)) phases.set(phase, []);
    phases.get(phase).push(task);
  });
  const summary = $("#phaseSummary");
  summary.replaceChildren();
  if (!state.tasks.length) {
    summary.append(makeElement("p", "muted-message", "No tasks yet. Add some only if they help you stay organized."));
    return;
  }
  phases.forEach((tasks, phase) => {
    const card = makeElement("div", "phase-card");
    card.append(
      makeElement("strong", "", phase),
      makeElement("span", "", `${tasks.filter((task) => task.status === "done").length} of ${tasks.length} done`)
    );
    summary.append(card);
  });
}

function renderTasks() {
  const list = $("#taskList");
  list.replaceChildren();
  if (!state.tasks.length) {
    list.append(makeElement("p", "muted-message", "No tasks. Add one if you want a checklist for this change."));
    return;
  }
  state.tasks.forEach((task, index) => {
    const row = $("#taskTemplate").content.firstElementChild.cloneNode(true);
    row.dataset.index = index;
    row.dataset.status = task.status;
    row.querySelectorAll("[data-field]").forEach((control) => {
      control.value = task[control.dataset.field] || "";
      control.addEventListener("input", handleTaskInput);
      control.addEventListener("change", handleTaskInput);
    });
    row.querySelector(".blocked-reason").hidden = task.status !== "blocked";
    row.querySelector('[data-action="up"]').disabled = index === 0;
    row.querySelector('[data-action="down"]').disabled = index === state.tasks.length - 1;
    row.querySelectorAll("[data-action]").forEach((button) => button.addEventListener("click", handleTaskAction));
    list.append(row);
  });
}

function handleTaskInput(event) {
  const row = event.target.closest(".task-row");
  const index = Number(row.dataset.index);
  const field = event.target.dataset.field;
  state.tasks[index][field] = event.target.value;
  if (field === "status") {
    if (event.target.value !== "blocked") state.tasks[index].blockedReason = "";
    renderTasks();
  }
  markDirty();
  renderProgress();
  renderTimeline();
}

async function handleTaskAction(event) {
  const row = event.target.closest(".task-row");
  const index = Number(row.dataset.index);
  const action = event.currentTarget.dataset.action;
  if (action === "remove") {
    const title = state.tasks[index].title || "this task";
    const ok = await askConfirm(`Delete “${title}”?`, "Delete task?");
    if (!ok) return;
    state.tasks.splice(index, 1);
  } else if (action === "up" && index > 0) {
    [state.tasks[index - 1], state.tasks[index]] = [state.tasks[index], state.tasks[index - 1]];
  } else if (action === "down" && index < state.tasks.length - 1) {
    [state.tasks[index + 1], state.tasks[index]] = [state.tasks[index], state.tasks[index + 1]];
  }
  markDirty();
  renderEditor();
}

function addTask() {
  const sequence = Date.now().toString(36);
  state.tasks.push({
    id: `task-${sequence}`,
    phase: "Other",
    title: "",
    status: "pending",
    owner: "",
    dueDate: "",
    notes: "",
    blockedReason: "",
    completedAt: "",
    order: state.tasks.length,
  });
  markDirty();
  renderEditor();
  const lastTitle = $("#taskList .task-row:last-child .task-title");
  lastTitle?.focus();
}

function projectFromForm() {
  const data = Object.fromEntries(new FormData($("#projectForm")).entries());
  data.tasks = state.tasks
    .filter((task) => task.title.trim())
    .map((task, order) => ({ ...task, title: task.title.trim(), order }));
  if (state.current) {
    data.createdAt = state.current.createdAt;
    data.history = state.current.history;
  }
  return data;
}

function buildAiBrief(project) {
  const sections = [
    ["Project", project.name],
    ["Problem", project.problem],
    ["Requirements", project.requirements],
    ["Context", project.context],
    ["Relevant files", project.relevantFiles],
    ["Acceptance criteria", project.acceptanceCriteria],
    ["Constraints", project.constraints],
    ["Hardware / fixture notes", project.hardwareNotes],
    ["Notes", project.notes],
  ];
  const lines = [
    "Please help with this cycle test machine code change.",
    "",
  ];
  sections.forEach(([label, value]) => {
    if (String(value || "").trim()) {
      lines.push(`## ${label}`);
      lines.push(String(value).trim());
      lines.push("");
    }
  });
  return lines.join("\n").trim() + "\n";
}

async function copyAiBrief() {
  const project = projectFromForm();
  if (!project.name.trim()) {
    showToast("Add a project name before copying the brief.", true);
    return;
  }
  const brief = buildAiBrief(project);
  if (brief.trim() === "Please help with this cycle test machine code change.") {
    showToast("Add a problem, requirements, or context first.", true);
    return;
  }
  try {
    await navigator.clipboard.writeText(brief);
    showToast("AI brief copied. Paste it into your chat when you are ready.");
  } catch (error) {
    showToast("Could not copy to the clipboard.", true);
  }
}

async function saveProject() {
  const form = $("#projectForm");
  if (!form.reportValidity()) return;
  const payload = projectFromForm();
  $("#saveButton").disabled = true;
  $("#saveState").textContent = "Saving…";
  try {
    const saved = state.isNew
      ? await api("/api/projects", { method: "POST", body: payload })
      : await api(`/api/projects/${encodeURIComponent(state.current.id)}`, { method: "PUT", body: payload });
    state.current = saved;
    state.tasks = structuredClone(saved.tasks);
    state.isNew = false;
    state.dirty = false;
    populateForm(saved);
    $("#editorProjectId").textContent = saved.id;
    $("#editorTitle").textContent = saved.name;
    showToast("Project saved.");
    renderEditor();
  } catch (error) {
    $("#saveState").textContent = "Save failed";
    showToast(error.message, true);
  } finally {
    $("#saveButton").disabled = false;
  }
}

function markDirty() {
  state.dirty = true;
  $("#saveState").textContent = "Unsaved changes";
}

function switchTab(tab) {
  state.activeTab = tab;
  $$(".tab").forEach((button) => button.classList.toggle("active", button.dataset.tab === tab));
  ["overview", "tasks", "timeline"].forEach((name) => {
    $(`#${name}Panel`).hidden = name !== tab;
  });
  if (tab === "timeline") renderTimeline();
}

function renderTimeline() {
  const upcoming = $("#upcomingList");
  upcoming.replaceChildren();
  const scheduled = state.tasks
    .filter((task) => task.dueDate && task.status !== "done")
    .sort((a, b) => a.dueDate.localeCompare(b.dueDate));
  if (!scheduled.length) {
    upcoming.append(makeElement("p", "muted-message", "Add due dates to tasks if you want a schedule."));
  } else {
    scheduled.forEach((task) => {
      const item = makeElement("div", "upcoming-item");
      const details = makeElement("div");
      details.append(makeElement("strong", "", task.title), makeElement("span", "", `${task.phase} · ${displayStatus(task.status)}`));
      item.append(details, makeElement("span", isOverdue(task) ? "overdue" : "", formatDate(task.dueDate)));
      upcoming.append(item);
    });
  }

  const history = $("#historyList");
  history.replaceChildren();
  const events = [...(state.current?.history || [])].reverse();
  if (!events.length) {
    history.append(makeElement("li", "muted-message", state.isNew ? "History begins after the project is saved." : "No activity recorded."));
  } else {
    events.forEach((entry) => {
      const item = makeElement("li", "history-item");
      item.append(makeElement("strong", "", entry.event), makeElement("time", "", formatTimestamp(entry.at)));
      history.append(item);
    });
  }
}

function downloadJson(project) {
  const blob = new Blob([`${JSON.stringify(project, null, 2)}\n`], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `${project.id || "project"}.json`;
  link.click();
  URL.revokeObjectURL(url);
}

async function duplicateProject() {
  try {
    const copy = await api(`/api/projects/${encodeURIComponent(state.current.id)}/duplicate`, {
      method: "POST",
      body: {},
    });
    $("#actionMenu").hidden = true;
    showToast("Project duplicated.");
    await openProject(copy.id);
  } catch (error) {
    showToast(error.message, true);
  }
}

async function archiveProject() {
  const ok = await askConfirm(
    `Archive “${state.current.name}”? You can still find it later by selecting Show archived.`,
    "Archive project?"
  );
  if (!ok) return;
  try {
    const archived = await api(`/api/projects/${encodeURIComponent(state.current.id)}/archive`, { method: "POST" });
    state.current = archived;
    state.tasks = structuredClone(archived.tasks);
    populateForm(archived);
    renderEditor();
    $("#archiveButton").textContent = "Project archived";
    $("#archiveButton").disabled = true;
    $("#actionMenu").hidden = true;
    showToast("Project archived.");
  } catch (error) {
    showToast(error.message, true);
  }
}

async function deleteProject() {
  $("#actionMenu").hidden = true;
  const ok = await askConfirm(
    `Delete “${state.current.name}”? This permanently removes the project file.`,
    "Delete project?"
  );
  if (!ok) return;
  try {
    await api(`/api/projects/${encodeURIComponent(state.current.id)}`, { method: "DELETE" });
    state.dirty = false;
    showToast("Project deleted.");
    showDashboard();
  } catch (error) {
    showToast(error.message, true);
  }
}

async function createBackup() {
  try {
    const result = await api("/api/backups", { method: "POST" });
    const link = document.createElement("a");
    link.href = result.downloadUrl;
    link.download = result.name;
    link.click();
    showToast("Backup created and downloaded.");
  } catch (error) {
    showToast(error.message, true);
  }
}

async function importProject(file) {
  if (!file) return;
  try {
    const payload = JSON.parse(await file.text());
    if (!payload.id || !payload.name) throw new Error("This file does not contain a valid project.");
    delete payload.progress;
    const imported = await api("/api/projects", { method: "POST", body: payload });
    showToast("Project imported.");
    await loadProjects();
    await openProject(imported.id);
  } catch (error) {
    showToast(`Import failed: ${error.message}`, true);
  } finally {
    $("#importFile").value = "";
  }
}

$("#newProjectButton").addEventListener("click", newProject);
$("#emptyNewButton").addEventListener("click", newProject);
$("#backButton").addEventListener("click", showDashboard);
$("#saveButton").addEventListener("click", saveProject);
$("#copyBriefButton").addEventListener("click", copyAiBrief);
$("#addTaskButton").addEventListener("click", addTask);
$("#searchInput").addEventListener("input", renderDashboard);
$("#showArchived").addEventListener("change", loadProjects);
$("#backupButton").addEventListener("click", createBackup);
$("#importButton").addEventListener("click", () => $("#importFile").click());
$("#importFile").addEventListener("change", (event) => importProject(event.target.files[0]));
$("#exportButton").addEventListener("click", () => downloadJson({ ...projectFromForm(), progress: taskProgress(state.tasks) }));
$("#duplicateButton").addEventListener("click", duplicateProject);
$("#archiveButton").addEventListener("click", archiveProject);
$("#deleteButton").addEventListener("click", deleteProject);
$("#confirmYes").addEventListener("click", () => closeConfirm(true));
$("#confirmNo").addEventListener("click", () => closeConfirm(false));
$("#confirmModal").addEventListener("click", (event) => {
  if (event.target === $("#confirmModal")) closeConfirm(false);
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !$("#confirmModal").hidden) closeConfirm(false);
});
$("#moreButton").addEventListener("click", () => {
  const menu = $("#actionMenu");
  menu.hidden = !menu.hidden;
  $("#moreButton").setAttribute("aria-expanded", String(!menu.hidden));
});
$$(".tab").forEach((button) => button.addEventListener("click", () => switchTab(button.dataset.tab)));
$("#projectForm").addEventListener("submit", (event) => event.preventDefault());
$("#projectForm").addEventListener("input", (event) => {
  if (!event.target.closest(".task-row")) markDirty();
  if (event.target.name === "name") $("#editorTitle").textContent = event.target.value || "Project intake";
});
window.addEventListener("beforeunload", (event) => {
  if (state.dirty) {
    event.preventDefault();
    event.returnValue = "";
  }
});
document.addEventListener("click", (event) => {
  if (!event.target.closest(".editor-actions")) $("#actionMenu").hidden = true;
});

loadProjects();
