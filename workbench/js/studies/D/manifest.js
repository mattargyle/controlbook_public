// Study D loader. List this study's scripts (paths relative to workbench/), in
// load order: the system file first, then the chapter files. Loaded while the
// page is parsing, so document.write places them before app.js.
WB.loadStudy('D', [
  'js/studies/D/system.js',
  'js/studies/D/models.js',
  'js/studies/D/pid.js',
  'js/studies/D/ss.js',
  'js/studies/D/freq.js',
]);
