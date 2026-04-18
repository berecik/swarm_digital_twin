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

document.addEventListener('DOMContentLoaded', () => {
  loadMissions();
  refreshStatus();
  setInterval(refreshStatus, 2000);
});
