const tabButtons = document.querySelectorAll("[data-media-tab]");
const panels = document.querySelectorAll("[data-media-panel]");

tabButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const target = button.dataset.mediaTab;

    tabButtons.forEach((tab) => {
      tab.classList.toggle("is-active", tab === button);
    });

    panels.forEach((panel) => {
      panel.classList.toggle("is-active", panel.dataset.mediaPanel === target);
    });
  });
});

document.querySelectorAll("[data-copy-target]").forEach((button) => {
  button.addEventListener("click", async () => {
    const target = document.getElementById(button.dataset.copyTarget);
    if (!target || !navigator.clipboard) return;

    const label = button.querySelector("span");
    const previousLabel = label ? label.textContent : "";

    await navigator.clipboard.writeText(target.innerText.trim());

    if (label) {
      label.textContent = "Copied";
      window.setTimeout(() => {
        label.textContent = previousLabel;
      }, 1400);
    }
  });
});
