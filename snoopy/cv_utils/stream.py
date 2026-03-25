import cv2
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# --- CONFIGURABLE PARAMETERS ---
WIDTH = 1280
HEIGHT = 720
FPS = 30
PORT = 8000
DEVICE_ID = 0  # Change to 1 if you have multiple USB cameras
# -------------------------------

class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    
                    # Encode as JPEG
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', len(buffer))
                    self.end_headers()
                    self.wfile.write(buffer.tobytes())
                    self.wfile.write(b'\r\n')
            except Exception as e:
                print(f"Connection closed: {e}")
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b'<html><body><img src="/stream.mjpg"></body></html>')

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads."""

# Initialize USB Camera
cap = cv2.VideoCapture(DEVICE_ID)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

try:
    server = ThreadedHTTPServer(('', PORT), StreamHandler)
    print(f"USB Camera streaming at http://localhost:{PORT}")
    server.serve_forever()
except KeyboardInterrupt:
    cap.release()
    server.socket.close()