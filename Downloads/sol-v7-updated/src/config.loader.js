// config.loader.js
// This file safely imports config with fallback for production builds

// Default configuration (used when config.js doesn't exist)
const DEFAULT_CONFIG = {
  passwords: {
    his: '26kylikh',
    hers: '3o3Pan',
  },
  users: {
    his: 'Kyaw',
    hers: 'Pan',
  }
};

let APP_CONFIG = DEFAULT_CONFIG;

// Try to load custom config if it exists
// Note: In production (Vercel), config.js won't exist and this will use defaults
// To change passwords in production, use Vercel Environment Variables:
// VITE_PASSWORD_HIS and VITE_PASSWORD_HERS

if (import.meta.env.VITE_PASSWORD_HIS || import.meta.env.VITE_PASSWORD_HERS) {
  // Use environment variables if provided
  APP_CONFIG = {
    passwords: {
      his: import.meta.env.VITE_PASSWORD_HIS || DEFAULT_CONFIG.passwords.his,
      hers: import.meta.env.VITE_PASSWORD_HERS || DEFAULT_CONFIG.passwords.hers,
    },
    users: {
      his: import.meta.env.VITE_USER_HIS || DEFAULT_CONFIG.users.his,
      hers: import.meta.env.VITE_USER_HERS || DEFAULT_CONFIG.users.hers,
    }
  };
}

export { APP_CONFIG };
