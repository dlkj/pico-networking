use core::fmt::Write;
use core::str::from_utf8;
use defmt::debug;
use embedded_hal::digital::v2::ToggleableOutputPin;
use heapless::Vec;
use smoltcp::{iface::SocketHandle, socket::tcp::Socket};

enum HttpRequest {
    Get,
    Unknown,
    ClientError,
}

enum HttpResponse {
    Ok,
    BadRequest,
    NotFound,
}

pub(crate) struct WebServer<'a, P: ToggleableOutputPin> {
    socket_handle: SocketHandle,
    pin: P,
    buffer: &'a mut Vec<u8, 512>,
}

impl<'a, P: ToggleableOutputPin> WebServer<'a, P> {
    pub fn new(socket_handle: SocketHandle, pin: P, buffer: &'a mut Vec<u8, 512>) -> Self {
        Self {
            socket_handle,
            pin,
            buffer,
        }
    }

    pub fn poll<'b, 'c: 'b>(&mut self, f: impl FnOnce(SocketHandle) -> &'b mut Socket<'c>) {
        let socket = f(self.socket_handle);
        if !socket.is_open() {
            socket.listen(80).unwrap();
            debug!("http: listening on 80");
        }

        if socket.can_recv() && socket.can_send() {
            let request = socket
                .recv(|buffer| (buffer.len(), request(buffer)))
                .unwrap();

            if let Some(request) = request {
                self.buffer.clear();
                let r = response(&request, self.buffer, &mut self.pin);

                match r {
                    HttpResponse::Ok => socket.send_slice(b"HTTP/1.1 200 OK\n"),
                    HttpResponse::BadRequest => socket.send_slice(b"HTTP/1.1 400 Bad Request\n"),
                    HttpResponse::NotFound => socket.send_slice(b"HTTP/1.1 404 Not Found\n"),
                }
                .unwrap();

                write!(
                    socket,
                    "Content-Type: text/html
Content-Length: {}\n\n",
                    self.buffer.len()
                )
                .unwrap();
                socket.send_slice(self.buffer).unwrap();
                socket.close();
            }
        } else if socket.may_send() && !socket.may_recv() {
            debug!("http: socket close");
            socket.close();
        }
    }
}

fn response(
    request: &HttpRequest,
    content: &mut Vec<u8, 512>,
    led: &mut impl ToggleableOutputPin,
) -> HttpResponse {
    match request {
        HttpRequest::Get => {
            write!(
                content,
                "<!DOCTYPE html>
<html><body><h1>Hello pico!</h1><p>This is some <i>html</i>!</p></body></html>"
            )
            .unwrap();
            led.toggle().ok().unwrap();
            HttpResponse::Ok
        }

        HttpRequest::Unknown => HttpResponse::NotFound,
        HttpRequest::ClientError => HttpResponse::BadRequest,
    }
}

fn request(buffer: &mut [u8]) -> Option<HttpRequest> {
    if buffer.is_empty() {
        None
    } else if let Ok(s) = from_utf8(buffer) {
        debug!("http: recv data: {:?}", s);

        if s.starts_with("GET / HTTP/1.1") {
            debug!("http: recv get /");
            Some(HttpRequest::Get)
        } else {
            debug!("http: recv unknown");
            Some(HttpRequest::Unknown)
        }
    } else {
        debug!("http: recv data: {:?}", buffer);
        Some(HttpRequest::ClientError)
    }
}
