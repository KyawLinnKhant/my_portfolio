// api/spotify/search.js
import axios from 'axios';

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

export default async function handler(req, res) {
  const cookies = parseCookies(req.headers.cookie);
  const accessToken = cookies.spotify_access_token;

  if (!accessToken) {
    return res.status(401).json({ error: 'Not authenticated with Spotify' });
  }

  const { q } = req.query;

  if (!q) {
    return res.status(400).json({ error: 'Search query is required' });
  }

  try {
    const response = await axios.get('https://api.spotify.com/v1/search', {
      headers: {
        'Authorization': `Bearer ${accessToken}`,
      },
      params: {
        q: q,
        type: 'track',
        limit: 20,
      },
    });

    const tracks = response.data.tracks.items.map(track => ({
      id: track.id,
      name: track.name,
      artists: track.artists,
      album: track.album,
      uri: track.uri,
      preview_url: track.preview_url,
    }));

    res.status(200).json({ tracks });
  } catch (error) {
    console.error('Spotify search error:', error.response?.data || error.message);
    
    if (error.response?.status === 401) {
      return res.status(401).json({ error: 'Spotify token expired. Please reconnect.' });
    }
    
    res.status(500).json({ 
      error: 'Failed to search Spotify',
      details: error.response?.data?.error?.message || error.message
    });
  }
}
