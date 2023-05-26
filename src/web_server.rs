use core::fmt::Write;
use core::str::from_utf8;
use defmt::debug;
use heapless::String;
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

pub(crate) struct WebServer {
    socket_handle: SocketHandle,
}

impl WebServer {
    pub fn new(socket_handle: SocketHandle) -> Self {
        Self { socket_handle }
    }

    pub fn poll<'a, 'b: 'a>(&mut self, f: impl FnOnce(SocketHandle) -> &'a mut Socket<'b>) {
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
                let mut content: Vec<u8, 256> = Vec::new();
                let r = response(&request, &mut content);

                match r {
                    HttpResponse::Ok => socket.send_slice(b"HTTP/1.1 200 OK\n"),
                    HttpResponse::BadRequest => socket.send_slice(b"HTTP/1.1 400 Bad Request\n"),
                    HttpResponse::NotFound => socket.send_slice(b"HTTP/1.1 404 Not Found\n"),
                }
                .unwrap();

                let mut response: String<256> = String::new();
                write!(
                    response,
                    "Connection: close
Content-Type: text/html
Content-Length: {}\n\n",
                    content.len()
                )
                .unwrap();
                socket.send_slice(response.as_bytes()).unwrap();
                socket.send_slice(content.as_slice()).unwrap();
                socket.close();
            }
        } else if socket.may_send() && !socket.may_recv() {
            debug!("http: socket close");
            socket.close();
        }
    }
}

fn response(request: &HttpRequest, content: &mut Vec<u8, 256>) -> HttpResponse {
    match request {
        HttpRequest::Get => {
            content
                .extend_from_slice(
                    b"<!DOCTYPE html>
<html><body><h1>Hello pico!</h1><p>This is some <i>html</i>!</p></body></html>",
                )
                .unwrap();
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
