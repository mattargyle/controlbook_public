// Study F loader. List this study's scripts (paths relative to workbench/), in
// load order: the system file first, then the chapter files. Loaded while the
// page is parsing, so document.write places them before app.js.
WB.loadStudy('F', [
  'js/studies/F/system.js',
  'js/studies/F/common.js',
  'js/studies/F/models.js',
  'js/studies/F/pid.js',
  'js/studies/F/ss.js',
  'js/studies/F/freq.js',
]);
