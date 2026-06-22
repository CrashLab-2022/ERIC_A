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

const carousels = document.querySelectorAll("[data-carousel]");

carousels.forEach((carousel) => {
  const slides = Array.from(carousel.querySelectorAll("[data-carousel-slide]"));
  const caption = carousel.querySelector("[data-carousel-caption]");
  const count = carousel.querySelector("[data-carousel-count]");
  const previousButton = carousel.querySelector("[data-carousel-previous]");
  const nextButton = carousel.querySelector("[data-carousel-next]");
  const viewport = carousel.querySelector(".carousel-viewport");
  let currentIndex = 0;
  let ignoreNextClick = false;
  let touchStartX = null;

  const showSlide = (index) => {
    currentIndex = (index + slides.length) % slides.length;

    slides.forEach((slide, slideIndex) => {
      slide.hidden = slideIndex !== currentIndex;
    });

    caption.textContent = slides[currentIndex].dataset.caption;
    count.textContent = `${currentIndex + 1} / ${slides.length}`;
  };

  previousButton.addEventListener("click", () => showSlide(currentIndex - 1));
  nextButton.addEventListener("click", () => showSlide(currentIndex + 1));
  viewport.addEventListener("click", () => {
    if (ignoreNextClick) {
      ignoreNextClick = false;
      return;
    }
    showSlide(currentIndex + 1);
  });

  carousel.addEventListener("keydown", (event) => {
    if (event.key === "ArrowLeft") showSlide(currentIndex - 1);
    if (event.key === "ArrowRight") showSlide(currentIndex + 1);
  });

  viewport.addEventListener(
    "touchstart",
    (event) => {
      touchStartX = event.changedTouches[0].clientX;
    },
    { passive: true },
  );

  viewport.addEventListener(
    "touchend",
    (event) => {
      if (touchStartX === null) return;

      const distance = event.changedTouches[0].clientX - touchStartX;
      if (Math.abs(distance) > 40) {
        ignoreNextClick = true;
        showSlide(currentIndex + (distance < 0 ? 1 : -1));
      }
      touchStartX = null;
    },
    { passive: true },
  );
});
