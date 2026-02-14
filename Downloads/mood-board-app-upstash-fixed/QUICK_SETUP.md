# 🚀 Quick Setup Guide - Upstash Version

## What You Need (5 minutes)

1. ✅ Upstash Redis account (free)
2. ✅ Vercel account (free)
3. ✅ GitHub account (free)

---

## Step 1: Get Upstash Redis (2 minutes)

1. Go to https://console.upstash.com/
2. Sign up with GitHub/Google
3. Click **"Create Database"**
4. Settings:
   - Name: `mood-board`
   - Type: **Regional** (free tier)
   - Region: Choose closest to you
   - Click **"Create"**
5. **Copy these values** (you'll need them):
   ```
   UPSTASH_REDIS_REST_URL=https://xxxxx.upstash.io
   UPSTASH_REDIS_REST_TOKEN=AxxxxxxxxxxxxxxxxxxxxxxxxxxxxQ
   ```

---

## Step 2: Push to GitHub (1 minute)

```bash
cd mood-board-app-upstash

# Initialize git
git init
git add .
git commit -m "Initial commit"

# Create repo on GitHub and push
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/mood-board.git
git push -u origin main
```

---

## Step 3: Deploy to Vercel (2 minutes)

1. Go to https://vercel.com
2. Sign in with GitHub
3. Click **"Add New..."** → **"Project"**
4. Select your `mood-board` repository
5. **Before deploying**, add Environment Variables:
   - Click **"Environment Variables"**
   - Add:
     ```
     UPSTASH_REDIS_REST_URL = your_url_from_step_1
     UPSTASH_REDIS_REST_TOKEN = your_token_from_step_1
     ```
6. Click **"Deploy"**

---

## Step 4: Done! 🎉

Your app will be live at:
```
https://mood-board-xyz123.vercel.app
```

Share this URL with your partner and start tracking your moods together!

---

## 🧪 Test It

1. Open the URL on your phone
2. Add a mood or song
3. Open the same URL on your computer
4. Wait 5 seconds → you'll see the update!

---

## 🎨 Customize

Edit these files to personalize:

**Names:**
- `components/MoodBoard.tsx` → Search for "Kyaw" and "Pan"

**Colors:**
- `app/globals.css` → Change the gradient and colors

**Moods:**
- `components/MoodBoard.tsx` → Edit the `moodEmojis` array

---

## ⚡ Pro Tips

1. **Custom Domain**: Add your own domain in Vercel settings
2. **PWA**: Make it installable as an app on phones
3. **Analytics**: Add Vercel Analytics to see usage
4. **Backups**: Export data regularly (coming soon feature)

---

## 🆘 Troubleshooting

**Deployment failed?**
- Check environment variables are set correctly
- Verify Upstash URL and token are correct

**Data not saving?**
- Check browser console (F12)
- Verify Upstash database is active
- Test API route: `https://your-app.vercel.app/api/data`

**AI not working?**
- Make sure you have data added (moods or songs)
- Check internet connection

---

## 📊 Monitor Your App

**Vercel Dashboard:**
- See deployments
- View logs
- Check analytics

**Upstash Dashboard:**
- Monitor database usage
- View stored data
- Check request counts

---

## 🔄 Update Your App

Made changes? Just push to GitHub:

```bash
git add .
git commit -m "Updated mood colors"
git push
```

Vercel will automatically redeploy! ✨

---

That's it! You now have a production-ready couples mood tracker with real-time sync! 🎊
