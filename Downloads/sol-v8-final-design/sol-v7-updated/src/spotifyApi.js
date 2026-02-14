// Spotify API helper - client-side search using Web API

const SPOTIFY_API_BASE = 'https://api.spotify.com/v1';
const SPOTIFY_TOKEN_URL = 'https://accounts.spotify.com/api/token';

// Get Spotify access token using client credentials
let cachedToken = null;
let tokenExpiry = null;

export async function getSpotifyToken() {
  // Check if we have a valid cached token
  if (cachedToken && tokenExpiry && Date.now() < tokenExpiry) {
    return cachedToken;
  }

  // Try to get token from our API endpoint
  try {
    const response = await fetch('/api/spotify/token');
    if (response.ok) {
      const data = await response.json();
      cachedToken = data.access_token;
      tokenExpiry = Date.now() + (data.expires_in * 1000) - 60000; // Refresh 1 min early
      return cachedToken;
    }
  } catch (error) {
    console.log('Token fetch failed:', error);
  }
  
  return null;
}

export async function searchTracks(query) {
  try {
    const token = await getSpotifyToken();
    
    if (!token) {
      console.log('No Spotify token available');
      // Return empty array - user needs to configure Spotify
      return [];
    }
    
    const encodedQuery = encodeURIComponent(query);
    const url = `${SPOTIFY_API_BASE}/search?q=${encodedQuery}&type=track&limit=10`;
    
    const response = await fetch(url, {
      headers: {
        'Authorization': `Bearer ${token}`
      }
    });
    
    if (response.ok) {
      const data = await response.json();
      return data.tracks.items.map(track => ({
        id: track.id,
        name: track.name,
        artist: track.artists.map(a => a.name).join(', '),
        album: track.album.name,
        image: track.album.images[2]?.url || track.album.images[0]?.url || '',
        uri: track.uri,
        external_url: track.external_urls.spotify
      }));
    } else {
      console.log('Spotify API error:', response.status);
      return [];
    }
  } catch (error) {
    console.error('Spotify search error:', error);
    return [];
  }
}

export function extractTrackId(spotifyUrl) {
  const match = spotifyUrl.match(/track\/([a-zA-Z0-9]+)/);
  return match ? match[1] : null;
}

// Generate a consistent ID for a search term (for embedding)
function generateSpotifyId(query) {
  // Use a simple hash to generate a consistent ID
  // This won't be a real Spotify ID but will work for embedding
  const hash = query.split('').reduce((acc, char) => {
    return char.charCodeAt(0) + ((acc << 5) - acc);
  }, 0);
  return Math.abs(hash).toString(36).padStart(22, '0').slice(0, 22);
}

export default {
  searchTracks,
  extractTrackId,
  getSpotifyToken
};
