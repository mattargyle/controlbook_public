// Study E loader. List this study's scripts (paths relative to workbench/), in
// load order: the system file first, then the chapter files. Loaded while the
// page is parsing, so document.write places them before app.js.
WB.loadStudy('E', [
  'js/studies/E/system.js',
  'js/studies/E/common.js',
  'js/studies/E/models.js',
  'js/studies/E/pid.js',
  'js/studies/E/ss.js',
  'js/studies/E/freq.js',
]);
