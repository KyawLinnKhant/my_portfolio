# Quick Setup Guide 🚀

## 🔐 Security First (2 minutes)

**Before anything else, set up your passwords:**

### Step 1: Configure Passwords
```bash
# Copy the example config
cp src/config.example.js src/config.js

# Edit src/config.js and set your passwords
# (This file is gitignored and won't be committed)
```

### Step 2: Configure Spotify (Optional)
```bash
# Copy the example env file
cp .env.example .env

# Edit .env and add your Spotify API credentials if you have them
# (This file is gitignored)
```

---

## Getting Started (5 minutes)

### Step 3: Install Dependencies
```bash
cd sol-v7-updated
npm install
```

### Step 4: Run Locally
```bash
npm run dev
```
Open http://localhost:5173

### Step 5: Test Camera
1. Click "Leave a Memory 💌"
2. Select "Photo"
3. Click "Take Photo"
4. Allow camera permissions
5. Take a photo!

---

## Deploy to GitHub + Vercel (10 minutes)

### Update GitHub Repository
```bash
# Navigate to project
cd sol-v7-updated

# Initialize git (if not already done)
git init

# Add all files
git add .

# Commit
git commit -m "v8: Camera + Aesthetic theme + Data fixes"

# Add your repository (replace with your URL)
git remote add origin https://github.com/KyawLinnKhant/to-pan.git

# Push
git branch -M main
git push -u origin main --force
```

### Deploy on Vercel

**Option 1: Auto-deploy (Recommended)**
1. Go to https://vercel.com
2. Import your GitHub repository
3. Vercel will auto-detect Vite
4. Click "Deploy"
5. Done! Your app is live

**Option 2: Vercel CLI**
```bash
npm i -g vercel
vercel login
vercel --prod
```

---

## Important Notes ⚠️

### Camera Permissions
- **HTTPS Required**: Camera only works on HTTPS (Vercel provides this)
- **User Permission**: Users must click "Allow" when prompted
- **Mobile**: Works on iOS Safari and Android Chrome

### Browser Support
- ✅ Chrome 53+
- ✅ Firefox 36+
- ✅ Safari 11+
- ✅ Edge 79+
- ❌ IE (not supported)

### Data Storage
- Uses artifact storage API (shared between users)
- Falls back to localStorage if unavailable
- Data persists across sessions

---

## Testing Checklist ✓

Before going live, test:

- [ ] Login with both passwords works
- [ ] Camera opens and captures photos
- [ ] Gallery upload still works
- [ ] Spotify search returns results
- [ ] Posts appear for both users
- [ ] Data persists after refresh
- [ ] Camera permissions prompt appears
- [ ] Mobile camera uses back camera
- [ ] Theme looks good on mobile
- [ ] All buttons work correctly

---

## Troubleshooting 🔧

### Camera Not Working
**Issue**: Camera permission denied
**Fix**: 
1. Check browser settings
2. Ensure HTTPS is enabled
3. Try different browser
4. Check camera is not used by another app

### Data Not Saving
**Issue**: Posts disappear after refresh
**Fix**:
1. Check browser console for errors
2. Try clearing cache
3. Ensure artifact storage is enabled
4. Check localStorage in DevTools

### Spotify Search Empty
**Issue**: No search results appear
**Fix**:
1. Check internet connection
2. Try different search terms
3. Use Spotify links instead
4. Check browser console

### Theme Not Updated
**Issue**: Still seeing old blue theme
**Fix**:
1. Clear browser cache (Ctrl+Shift+R)
2. Hard reload the page
3. Check App.css is properly loaded

---

## File Structure

```
sol-v7-updated/
├── src/
│   ├── App.jsx          ← Main component (camera logic here)
│   ├── App.css          ← New aesthetic theme
│   ├── spotifyApi.js    ← Spotify integration
│   ├── components/
│   │   └── Calendar.jsx
│   ├── main.jsx
│   └── index.css
├── public/
│   └── pansy.png        ← Logo
├── api/
│   └── spotify/         ← API routes (Vercel serverless)
├── package.json
├── vite.config.js
└── README.md
```

---

## Quick Commands

```bash
# Development
npm run dev          # Start dev server
npm run build        # Build for production
npm run preview      # Preview production build

# Git
git status           # Check changes
git add .            # Stage all changes
git commit -m "msg"  # Commit with message
git push             # Push to GitHub

# Vercel
vercel               # Deploy to preview
vercel --prod        # Deploy to production
```

---

## Need Help?

1. **Camera Issues**: Check HTTPS and permissions
2. **Data Issues**: Check browser console
3. **Theme Issues**: Clear cache and hard reload
4. **Spotify Issues**: Try direct links

---

**Ready to Deploy?** Follow the GitHub + Vercel steps above! 🚀

**Version**: 8.0
**Last Updated**: February 14, 2026
