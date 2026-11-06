// %% live model and stream status

(() => {
  const panel = document.createElement("div");
  Object.assign(panel.style, {
    position: "fixed", top: "12px", left: "12px", padding: "14px",
    background: "rgba(255,255,255,.94)", font: "14px sans-serif",
    borderRadius: "8px", maxWidth: "440px", zIndex: "10"
  });
  document.body.appendChild(panel);
  panel.textContent = "In attesa dei dati del monitor…";
  let lastGeneratedAt = null;
  let lastChange = performance.now();
  let maximumAge = null;

  async function update() {
    const response = await fetch("snapshot.json", {cache: "no-store"}).catch(() => null);
    if (!response || !response.ok) {
      panel.textContent = "Monitor non disponibile — nessun dato aggiornato";
      panel.style.color = "#a32121";
      return;
    }
    const snapshot = await response.json();
    maximumAge = snapshot.maximum_age;
    if (snapshot.generated_at !== lastGeneratedAt) {
      lastGeneratedAt = snapshot.generated_at;
      lastChange = performance.now();
    }
    const disconnected = performance.now() - lastChange > snapshot.maximum_age * 1000;
    const streams = snapshot.streams.__items__;
    const current = !disconnected && streams.every(item => item.condition === "current");
    panel.style.color = current ? "#175c30" : "#a32121";
    panel.replaceChildren();
    const title = document.createElement("strong");
    title.textContent = disconnected ? "Monitor interrotto — posa congelata" : "TIAGo · sola lettura";
    panel.appendChild(title);
    for (const stream of streams) {
      const row = document.createElement("div");
      const age = stream.age === null ? "—" : `${stream.age.toFixed(2)} s`;
      row.textContent = `${stream.topic}: ${disconnected ? "disconnected" : stream.condition} (${age})`;
      panel.appendChild(row);
    }
    const geometry = document.createElement("div");
    const error = snapshot.lidar_translation_error;
    geometry.textContent = `Scarto TF lidar: ${error === null ? "—" : `${(error * 1000).toFixed(1)} mm`} · Contatti ≤ 0: ${snapshot.penetrating_contacts ?? "—"}`;
    panel.appendChild(geometry);
    if (typeof scene === "undefined") return;
    for (const pose of snapshot.poses.__items__) {
      const object = scene.getObjectByName(pose.node_name);
      if (!object) continue;
      object.matrixAutoUpdate = false;
      object.matrix.fromArray(pose.matrix.__items__);
      object.matrixWorldNeedsUpdate = true;
    }
    if (typeof render === "function" && typeof camera !== "undefined") render();
  }
  setInterval(update, 500);
  setInterval(() => {
    if (maximumAge !== null && performance.now() - lastChange > maximumAge * 1000) {
      panel.textContent = "Monitor interrotto — posa congelata";
      panel.style.color = "#a32121";
    }
  }, 250);
  update();
})();
