// Launcher page logic — loads /api/missions, renders the card grid,
// polls /api/status for the hero chip, and shows a command-reveal modal
// when a card's START button is pressed.

const grid = document.getElementById('mission-grid');
const heroDot = document.getElementById('hero-status-dot');
const heroValue = document.getElementById('hero-status-value');
const modal = document.getElementById('launch-modal');
const modalTitle = document.getElementById('launch-title');
const modalCmd = document.getElementById('launch-command');
const modalCopy = document.getElementById('launch-copy');

function escapeHtml(s) {
  return String(s)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;');
}

function renderCard(m) {
  const card = document.createElement('article');
  card.className = 'mission-card';
  const tier = (m.tier || 'free').toLowerCase();
  const disabled = Boolean(m.disabled);

  const thumbInner = m.thumbnail
    ? `<img src="${escapeHtml(m.thumbnail)}" alt="${escapeHtml(m.title || m.id || '')}" onerror="this.style.display='none'" />`
    : '';

  card.innerHTML = `
    <div class="mission-thumb">
      ${thumbInner}
      <span class="tier-pill tier-${tier}">${tier.toUpperCase()}</span>
      ${disabled ? `
        <div class="lock-overlay">
          <div class="lock-icon">&#128274;</div>
          <div class="lock-label">PRO ACCESS REQUIRED</div>
        </div>` : ''}
    </div>
    <div class="mission-body">
      <h3 class="mission-title">${escapeHtml(m.title || m.id || 'Mission')}</h3>
      <p class="mission-desc">${escapeHtml(m.description || '')}</p>
    </div>
    <div class="mission-footer">
      ${disabled
        ? `<button class="btn btn-blue">GET PRO</button>`
        : `<button class="btn btn-green" data-cmd="${escapeHtml(m.start_command || '')}" data-title="${escapeHtml(m.title || '')}">
              <span>&#9658;</span> START
           </button>`}
    </div>
  `;

  const startBtn = card.querySelector('.btn-green');
  if (startBtn) {
    startBtn.addEventListener('click', (ev) => {
      ev.stopPropagation();
      openLaunchModal(startBtn.dataset.title || '', startBtn.dataset.cmd || '');
    });
  }
  return card;
}

function openLaunchModal(title, cmd) {
  modalTitle.textContent = title ? `Launch: ${title}` : 'Launch mission';
  modalCmd.textContent = cmd;
  modal.classList.remove('hidden');
}

function closeLaunchModal() {
  modal.classList.add('hidden');
}

modal.addEventListener('click', (ev) => {
  if (ev.target.hasAttribute('data-close-modal')) closeLaunchModal();
});
document.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape') closeLaunchModal();
});
modalCopy.addEventListener('click', () => {
  const txt = modalCmd.textContent || '';
  navigator.clipboard?.writeText(txt).then(() => {
    modalCopy.textContent = 'Copied!';
    setTimeout(() => (modalCopy.textContent = 'Copy command'), 1400);
  });
});

async function loadMissions() {
  try {
    const res = await fetch('/api/missions');
    const missions = await res.json();
    grid.innerHTML = '';
    if (!Array.isArray(missions) || missions.length === 0) {
      grid.innerHTML = '<p style="color:var(--fg-1)">No missions defined.</p>';
      return;
    }
    for (const m of missions) grid.appendChild(renderCard(m));
  } catch (err) {
    grid.innerHTML = `<p style="color:var(--status-bad)">Failed to load missions: ${err}</p>`;
  }
}

async function refreshStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    if (data.connected) {
      heroDot.style.background = 'var(--status-good)';
      heroDot.style.boxShadow = '0 0 10px rgba(34,197,94,0.8)';
      heroValue.textContent = `CONNECTED · ${data.sample_count} samples`;
    } else if ((data.sample_count || 0) > 0) {
      heroDot.style.background = 'var(--status-warn)';
      heroValue.textContent = `BUFFERED · ${data.sample_count} samples`;
    } else {
      heroDot.style.background = 'var(--status-bad)';
      heroValue.textContent = 'DISCONNECTED';
    }
  } catch (err) {
    heroDot.style.background = 'var(--status-bad)';
    heroValue.textContent = 'OFFLINE';
  }
}

// ── Replay file picker ─────────────────────────────────────────────
// File names come from GET /api/files (server filesystem listing) and
// are escaped via escapeHtml() before insertion — no XSS risk.

const replayGrid = document.getElementById('replay-grid');

function renderFileCard(f) {
  const card = document.createElement('article');
  card.className = 'mission-card';
  const sizeKB = (f.size / 1024).toFixed(0);

  // Build card using DOM methods for safety; only escaped text in innerHTML.
  const thumb = document.createElement('div');
  thumb.className = 'mission-thumb';
  thumb.style.cssText = 'display:flex;align-items:center;justify-content:center;font-size:3rem;background:var(--bg-2)';
  thumb.textContent = f.type === 'npz' ? '\u{1F4CA}' : '\u{1F4C4}';

  const body = document.createElement('div');
  body.className = 'mission-body';
  const h3 = document.createElement('h3');
  h3.className = 'mission-title';
  h3.textContent = f.name;
  const desc = document.createElement('p');
  desc.className = 'mission-desc';
  desc.textContent = `${sizeKB} KB \u00B7 ${f.type.toUpperCase()}`;
  body.appendChild(h3);
  body.appendChild(desc);

  const footer = document.createElement('div');
  footer.className = 'mission-footer';
  const btn = document.createElement('button');
  btn.className = 'btn btn-green replay-btn';
  btn.dataset.path = f.path;
  btn.textContent = '\u25B6 REPLAY';
  footer.appendChild(btn);

  card.appendChild(thumb);
  card.appendChild(body);
  card.appendChild(footer);

  btn.addEventListener('click', async (ev) => {
    ev.stopPropagation();
    btn.textContent = 'Loading\u2026';
    btn.disabled = true;
    try {
      const res = await fetch(`/api/load?path=${encodeURIComponent(f.path)}`, {
        method: 'POST',
      });
      if (!res.ok) {
        const err = await res.json().catch(() => ({}));
        alert(`Failed to load: ${err.detail || res.statusText}`);
        return;
      }
      window.location.href = '/live';
    } catch (err) {
      alert(`Error: ${err}`);
    } finally {
      btn.textContent = '\u25B6 REPLAY';
      btn.disabled = false;
    }
  });
  return card;
}

async function loadFiles() {
  if (!replayGrid) return;
  try {
    const res = await fetch('/api/files');
    const files = await res.json();
    replayGrid.textContent = '';
    if (!Array.isArray(files) || files.length === 0) {
      const p = document.createElement('p');
      p.style.color = 'var(--fg-1)';
      p.textContent = 'No flight data files found. Run a simulation first.';
      replayGrid.appendChild(p);
      return;
    }
    for (const f of files) replayGrid.appendChild(renderFileCard(f));
  } catch (err) {
    const p = document.createElement('p');
    p.style.color = 'var(--status-bad)';
    p.textContent = `Failed to list files: ${err}`;
    replayGrid.appendChild(p);
  }
}

// ── Init ──────────────────────────────────────────────────────────

document.addEventListener('DOMContentLoaded', () => {
  loadMissions();
  loadFiles();
  refreshStatus();
  setInterval(refreshStatus, 2000);
});
