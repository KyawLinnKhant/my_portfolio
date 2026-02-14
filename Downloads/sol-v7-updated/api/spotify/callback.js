// api/spotify/callback.js
import axios from 'axios';

const CLIENT_ID = process.env.SPOTIFY_CLIENT_ID;
const CLIENT_SECRET = process.env.SPOTIFY_CLIENT_SECRET;
const REDIRECT_URI = process.env.SPOTIFY_REDIRECT_URI;

export default async function handler(req, res) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { code } = req.body;

  if (!code) {
    return res.status(400).json({ error: 'Authorization code is required' });
  }

  try {
    const response = await axios.post(
      'https://accounts.spotify.com/api/token',
      new URLSearchParams({
        grant_type: 'authorization_code',
        code: code,
        redirect_uri: REDIRECT_URI,
      }),
      {
        headers: {
          'Content-Type': 'application/x-www-form-urlencoded',
          'Authorization': 'Basic ' + Buffer.from(CLIENT_ID + ':' + CLIENT_SECRET).toString('base64'),
        },
      }
    );

    const { access_token, refresh_token, expires_in } = response.data;

    // Store tokens in cookie (in production, use secure session management)
    res.setHeader('Set-Cookie', [
      `spotify_access_token=${access_token}; Path=/; HttpOnly; SameSite=Lax; Max-Age=${expires_in}`,
      `spotify_refresh_token=${refresh_token}; Path=/; HttpOnly; SameSite=Lax; Max-Age=31536000`, // 1 year
    ]);

    res.status(200).json({ success: true });
  } catch (error) {
    console.error('Spotify callback error:', error.response?.data || error.message);
    res.status(500).json({ 
      error: 'Failed to exchange authorization code',
      details: error.response?.data?.error_description || error.message
    });
  }
}
