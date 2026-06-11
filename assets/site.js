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
