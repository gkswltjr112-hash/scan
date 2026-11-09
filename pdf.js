/* 문서 스캐너 - 외부 라이브러리 없는 간단한 PDF 생성기 (JPEG 이미지를 페이지로 넣음) */
(function (root) {
  'use strict';

  function latin1(s) {
    var u = new Uint8Array(s.length);
    for (var i = 0; i < s.length; i++) u[i] = s.charCodeAt(i) & 255;
    return u;
  }
  function num(n) { return (Math.round(n * 100) / 100).toString(); }
  function utf16hex(s) {
    var hex = 'FEFF';
    for (var i = 0; i < s.length; i++) hex += ('0000' + s.charCodeAt(i).toString(16)).slice(-4).toUpperCase();
    return '<' + hex + '>';
  }
  function pad2(n) { return ('0' + n).slice(-2); }

  /* pages: [{ jpeg: Uint8Array, w: 픽셀, h: 픽셀 }]
     opts : { size: 'a4' | 'fit', title: '문자열' }  →  Blob(application/pdf) */
  function buildPdf(pages, opts) {
    opts = opts || {};
    var mode = opts.size || 'a4';
    var chunks = [], offset = 0, offs = [];
    function push(x) { var u = typeof x === 'string' ? latin1(x) : x; chunks.push(u); offset += u.length; }
    function obj(n, body) { offs[n] = offset; push(n + ' 0 obj\n' + body + '\nendobj\n'); }

    push('%PDF-1.4\n%\xE2\xE3\xCF\xD3\n');
    var count = pages.length, infoNum = 3 + 3 * count, kids = [], i;
    for (i = 0; i < count; i++) kids.push((3 + 3 * i) + ' 0 R');
    obj(1, '<< /Type /Catalog /Pages 2 0 R >>');
    obj(2, '<< /Type /Pages /Kids [' + kids.join(' ') + '] /Count ' + count + ' >>');

    for (i = 0; i < count; i++) {
      var pg = pages[i], pn = 3 + 3 * i, cn = pn + 1, im = pn + 2;
      var pw, ph, iw, ih, ix, iy;
      if (mode === 'a4') {
        var land = pg.w > pg.h;
        pw = land ? 841.89 : 595.28; ph = land ? 595.28 : 841.89;
        var s = Math.min(pw / pg.w, ph / pg.h);
        iw = pg.w * s; ih = pg.h * s; ix = (pw - iw) / 2; iy = (ph - ih) / 2;
      } else {                       // 이미지 크기 그대로 (여백 없이, 긴 변이 A4 정도가 되게)
        var dpi = Math.max(150, Math.max(pg.w, pg.h) / 11.69);
        pw = iw = pg.w * 72 / dpi; ph = ih = pg.h * 72 / dpi; ix = 0; iy = 0;
      }
      obj(pn, '<< /Type /Page /Parent 2 0 R /MediaBox [0 0 ' + num(pw) + ' ' + num(ph) + '] ' +
              '/Resources << /XObject << /Im0 ' + im + ' 0 R >> >> /Contents ' + cn + ' 0 R >>');
      var content = 'q ' + num(iw) + ' 0 0 ' + num(ih) + ' ' + num(ix) + ' ' + num(iy) + ' cm /Im0 Do Q';
      obj(cn, '<< /Length ' + content.length + ' >>\nstream\n' + content + '\nendstream');
      offs[im] = offset;
      push(im + ' 0 obj\n<< /Type /XObject /Subtype /Image /Width ' + pg.w + ' /Height ' + pg.h +
           ' /ColorSpace /DeviceRGB /BitsPerComponent 8 /Filter /DCTDecode /Length ' + pg.jpeg.length + ' >>\nstream\n');
      push(pg.jpeg);
      push('\nendstream\nendobj\n');
    }

    var now = new Date();
    var date = 'D:' + now.getFullYear() + pad2(now.getMonth() + 1) + pad2(now.getDate()) +
               pad2(now.getHours()) + pad2(now.getMinutes()) + pad2(now.getSeconds());
    obj(infoNum, '<< ' + (opts.title ? '/Title ' + utf16hex(opts.title) + ' ' : '') +
                 '/Producer (Document Scanner) /CreationDate (' + date + ') >>');

    var total = infoNum + 1, xrefPos = offset, x = 'xref\n0 ' + total + '\n0000000000 65535 f \n';
    for (i = 1; i < total; i++) x += ('0000000000' + offs[i]).slice(-10) + ' 00000 n \n';
    x += 'trailer\n<< /Size ' + total + ' /Root 1 0 R /Info ' + infoNum + ' 0 R >>\nstartxref\n' + xrefPos + '\n%%EOF\n';
    push(x);
    return new Blob(chunks, { type: 'application/pdf' });
  }

  var api = { buildPdf: buildPdf };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.PdfBuilder = api;
})(typeof self !== 'undefined' ? self : this);
