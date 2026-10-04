// Study B loader. List this study's scripts (paths relative to workbench/), in
// load order: the system file first, then the chapter files. Loaded while the
// page is parsing, so document.write places them before app.js.
WB.loadStudy('B', [
  'js/studies/B/system.js',
  'js/studies/B/lib.js',
  'js/studies/B/models.js',
  'js/studies/B/slc.js',
  'js/studies/B/ss.js',
  'js/studies/B/freq.js',
]);
