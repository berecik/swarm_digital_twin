document.addEventListener('DOMContentLoaded', () => {
    const grid = document.getElementById('mission-grid');

    fetch('/api/missions')
        .then(res => res.json())
        .then(missions => {
            missions.forEach(mission => {
                const card = document.createElement('div');
                card.className = 'card';
                card.innerHTML = `
                    <img src="${mission.thumbnail}" alt="${mission.name}">
                    <div class="card-content">
                        <h2>${mission.name}</h2>
                        <p>${mission.description}</p>
                    </div>
                    <div class="card-footer">
                        <a href="/live?mission=${mission.id}" class="btn">Monitor Live</a>
                    </div>
                `;
                card.onclick = () => {
                    window.location.href = `/live?mission=${mission.id}`;
                };
                grid.appendChild(card);
            });
        });
});
