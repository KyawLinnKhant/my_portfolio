// api/spotify/login.js
const CLIENT_ID = process.env.SPOTIFY_CLIENT_ID;
const REDIRECT_URI = process.env.SPOTIFY_REDIRECT_URI;

export default function handler(req, res) {
  if (!CLIENT_ID || !REDIRECT_URI) {
    return res.status(500).json({ 
      error: 'Spotify credentials not configured. Please set SPOTIFY_CLIENT_ID and SPOTIFY_REDIRECT_URI in Vercel environment variables.' 
    });
  }

  const scopes = [
    'user-read-private',
    'user-read-email',
    'user-library-read',
    'playlist-read-private',
  ];

  const authUrl = `https://accounts.spotify.com/authorize?${new URLSearchParams({
    response_type: 'code',
    client_id: CLIENT_ID,
    scope: scopes.join(' '),
    redirect_uri: REDIRECT_URI,
    show_dialog: 'true'
  })}`;

  res.status(200).json({ url: authUrl });
}
