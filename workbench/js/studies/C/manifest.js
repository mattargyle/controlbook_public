// Study C loader. List this study's scripts (paths relative to workbench/), in
// load order: the system file first, then the chapter files. Loaded while the
// page is parsing, so document.write places them before app.js.
WB.loadStudy('C', [
  'js/studies/C/system.js',
  'js/studies/C/models.js',
  'js/studies/C/pid.js',
  'js/studies/C/ss.js',
  'js/studies/C/freq.js',
]);
