import base64, json, pathlib
here = pathlib.Path(__file__).parent
app = (here / 'app.html').read_text()
engine = (here / 'engine.js').read_text()
uri = 'data:video/webm;base64,' + base64.b64encode((here / 'clips/sample.webm').read_bytes()).decode()
truth = json.loads((here / 'clips/y9_s4_140.json').read_text())
pts = [[round(x * 1280 / 1920, 1), round(y * 720 / 1080, 1)] for x, y in truth['calib_points_exact']]
out = app.replace('/*ENGINE*/', engine).replace('__SAMPLE_URI__', uri).replace('__SAMPLE_PTS__', json.dumps(pts))
(here / 'dist').mkdir(exist_ok=True)
(here / 'dist/cricket-radar.html').write_text(out)
# standalone copy: a full document so it runs when opened or hosted on its own
(here / 'dist/cricket-radar-standalone.html').write_text(
  '<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">'
  '<style>[hidden]{display:none!important}body{margin:0}:root{padding-top:env(safe-area-inset-top,0px);padding-bottom:env(safe-area-inset-bottom,0px)}</style></head><body>'
  + out + '</body></html>')
print(len(out))
