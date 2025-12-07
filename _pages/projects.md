---
layout: default
title: Sajni Shah - Portfolio
permalink: /projects/
---

<!-- MathJax for rendering LaTeX in Markdown -->
<script>
  MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\(', '\\)']],
      displayMath: [['$$', '$$'], ['\\[', '\\]']],
      processEscapes: true,
      processEnvironments: true
    },
    options: {
      skipHtmlTags: ['noscript', 'style', 'textarea', 'pre', 'code']
    }
  };
</script>
<script async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>


<style>
  /* Page-local styles (no global CSS changes) */
  .project-gallery{
    display:grid;
    grid-template-columns:repeat(auto-fit, minmax(260px, 320px));
    justify-content:center;     /* centers the tracks */
    gap:1.25rem;
    margin:2rem auto 4rem;
    padding:0 1rem;
  }
  .gallery-item{ width:100%; text-align:center; }
  .gallery-item a{ display:block; text-decoration:none; color:inherit; }
  .gallery-item img{ display:block; width:100%; height:auto; border-radius:10px; }
  .gallery-item p{ margin-top:.5rem; font-weight:400; }

</style>

<div class="project-gallery">
  {% for project in site.projects %}
    <div class="gallery-item">
      <a href="{{ project.url | relative_url }}">
        <img src="{{ project.image | relative_url }}" alt="{{ project.title }}" />
        <p>{{ project.title }}</p>
      </a>
    </div>
  {% endfor %}
</div>
