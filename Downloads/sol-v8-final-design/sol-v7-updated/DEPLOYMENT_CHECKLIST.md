# Pre-Deployment Checklist ✅

## Before You Deploy - Critical Steps

### 1. Security Configuration ✅

- [ ] Created `src/config.js` from `src/config.example.js`
- [ ] Updated passwords in `src/config.js`
- [ ] Verified `src/config.js` is in `.gitignore`
- [ ] Created `.env` from `.env.example` (optional)
- [ ] Added Spotify credentials to `.env` (if using)
- [ ] Verified `.env` is in `.gitignore`

### 2. Test Locally ✅

```bash
npm install
npm run dev
```

- [ ] App runs without errors
- [ ] Can login with configured passwords
- [ ] Camera opens and captures photos
- [ ] Gallery upload works
- [ ] Spotify search returns results
- [ ] Posts save and persist after refresh
- [ ] Both users can see shared posts

### 3. Git Status Check ✅

```bash
git status
```

**Verify these files are NOT listed:**
- [ ] `src/config.js` (should be gitignored)
- [ ] `.env` (should be gitignored)
- [ ] `node_modules/` (should be gitignored)

**Verify these files ARE included:**
- [ ] `src/config.example.js`
- [ ] `.env.example`
- [ ] `.gitignore`
- [ ] All source code files
- [ ] `SECURITY.md`

### 4. Commit and Push ✅

```bash
git init
git add .
git commit -m "v8: Camera + Aesthetic theme + Data fixes (passwords secured)"
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPO.git
git branch -M main
git push -u origin main
```

**After pushing:**
- [ ] Go to GitHub repo
- [ ] Verify `src/config.js` is NOT visible
- [ ] Verify `.env` is NOT visible
- [ ] Check that only `config.example.js` exists

### 5. Vercel Deployment ✅

#### Option A: Automatic (Recommended)
1. [ ] Go to https://vercel.com
2. [ ] Import your GitHub repository
3. [ ] Vercel auto-detects Vite settings
4. [ ] Click "Deploy"

#### Option B: CLI
```bash
npm i -g vercel
vercel login
vercel --prod
```

#### Environment Variables (Optional)
If using Spotify API:
1. [ ] Go to Vercel Project Settings
2. [ ] Navigate to Environment Variables
3. [ ] Add:
   - `SPOTIFY_CLIENT_ID`
   - `SPOTIFY_CLIENT_SECRET`
   - `SPOTIFY_REDIRECT_URI`

### 6. Post-Deployment Testing ✅

Visit your deployed site and test:

- [ ] HTTPS is enabled (required for camera)
- [ ] Login works with your passwords
- [ ] Camera permission prompt appears
- [ ] Camera captures photos successfully
- [ ] Gallery upload works
- [ ] Spotify search works
- [ ] Posts save and sync between users
- [ ] Theme looks correct (peach/coral)
- [ ] Mobile camera uses back camera
- [ ] No console errors

### 7. Share with Users ✅

- [ ] Send app URL (e.g., `https://your-app.vercel.app`)
- [ ] Share passwords through SECURE channel:
  - Signal/WhatsApp (encrypted)
  - Password manager
  - In-person
  - **NOT through Git/GitHub**
- [ ] Inform about camera permissions needed
- [ ] Inform that HTTPS is required

---

## Troubleshooting

### "Config not found" error
```bash
# Create config.js from example
cp src/config.example.js src/config.js
# Then edit with your passwords
```

### Config.js visible in git status
```bash
# Remove from staging
git rm --cached src/config.js
# Verify it's in .gitignore
cat .gitignore | grep config.js
```

### Passwords not working after deployment
- Check that `config.js` exists in your local project
- Verify the passwords match what you set
- Hard refresh browser (Ctrl+Shift+R)

### Camera not working on deployed site
- Ensure site is using HTTPS (Vercel provides this)
- Check browser permissions
- Try different browser

---

## Quick Reference

### Files to NEVER commit:
```
src/config.js
.env
node_modules/
.vercel/
```

### Files to ALWAYS commit:
```
src/config.example.js
.env.example
.gitignore
SECURITY.md
```

### Safe to share publicly:
- GitHub repository URL
- Vercel app URL
- Source code (without config.js)
- Documentation files

### Keep private:
- Passwords in config.js
- API keys in .env
- Any credentials

---

## Final Verification

Before considering deployment complete:

```bash
# 1. Check git doesn't have secrets
git log --all --full-history -- src/config.js
# Should return nothing

# 2. Check GitHub repo online
# Visit: https://github.com/YOUR_USERNAME/YOUR_REPO
# Search for "26kylikh" or "3o3Pan"
# Should find: 0 results

# 3. Test deployed app
# Open in incognito/private window
# Login with passwords
# All features work
```

✅ All checks passed? You're ready to go! 🚀

---

**Last Updated**: February 14, 2026
**Version**: 8.0 (Secured)
