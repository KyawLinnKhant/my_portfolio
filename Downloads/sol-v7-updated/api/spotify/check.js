// api/spotify/check.js
function parseCookies(cookieHeader) {
  const cookies = {};
  if (cookieHeader) {
    cookieHeader.split(';').forEach(cookie => {
      const [name, value] = cookie.trim().split('=');
      cookies[name] = value;
    });
  }
  return cookies;
}

export default function handler(req, res) {
  const cookies = parseCookies(req.headers.cookie);
  const accessToken = cookies.spotify_access_token;

  res.status(200).json({ 
    connected: !!accessToken 
  });
}
