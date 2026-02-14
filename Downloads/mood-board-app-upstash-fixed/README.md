# Mood Board - Couples Mood Tracker (Upstash Version)

A beautiful web app for couples to track their moods, share music, and create memories together - with **real-time syncing** using Upstash Redis and AI-powered insights!

## ✨ Key Features

- 📅 **Mood Calendar** - Track daily emotions for both partners
- 🎵 **Shared Music Library** - Build a playlist together
- 📸 **Photo Gallery** - Upload and share special moments
- 🤖 **AI Insights** - Relationship insights powered by Claude AI
- 🔄 **Real-time Sync** - Changes sync across devices automatically
- ☁️ **Cloud Storage** - Data stored in Upstash Redis (persistent & fast)

## 🚀 What's Special About This Version?

**Unlike the localStorage version**, this app uses **Upstash Redis** which means:

✅ **Shared Data** - Both partners see the same data in real-time
✅ **Multi-Device** - Access from phone, tablet, or computer
✅ **Persistent** - Data never gets lost (stored in the cloud)
✅ **Auto-Sync** - Updates every 5 seconds automatically
✅ **Scalable** - Can handle lots of data without slowing down

## 🛠️ Tech Stack

- **Next.js 14** - React framework
- **TypeScript** - Type safety
- **Upstash Redis** - Cloud database (serverless)
- **Vercel** - Hosting & deployment
- **Lucide React** - Beautiful icons
- **Claude AI** - Relationship insights

---

## 📦 Quick Setup Guide

### 1. Get Upstash Redis (FREE)

1. Go to [Upstash Console](https://console.upstash.com/)
2. Sign up (free account)
3. Click "Create Database"
4. Choose:
   - Name: `mood-board`
   - Type: **Regional** (faster & free)
   - Region: Choose closest to you
5. Click "Create"
6. Copy your credentials:
   - `UPSTASH_REDIS_REST_URL`
   - `UPSTASH_REDIS_REST_TOKEN`

### 2. Deploy to Vercel

#### Option A: Using Vercel Website (Recommended)

1. Push your code to GitHub:
```bash
git init
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/mood-board.git
git push -u origin main
```

2. Go to [Vercel](https://vercel.com) and import your repo

3. **Add Environment Variables** in Vercel:
   - Click "Environment Variables"
   - Add `UPSTASH_REDIS_REST_URL` → paste your URL
   - Add `UPSTASH_REDIS_REST_TOKEN` → paste your token

4. Click "Deploy"

5. Done! 🎉

#### Option B: Using Vercel CLI

1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Deploy:
```bash
vercel
```

3. Add environment variables:
```bash
vercel env add UPSTASH_REDIS_REST_URL
vercel env add UPSTASH_REDIS_REST_TOKEN
```

4. Redeploy:
```bash
vercel --prod
```

### 3. Local Development (Optional)

If you want to test locally first:

1. Install dependencies:
```bash
npm install
```

2. Create `.env.local` file:
```bash
cp .env.local.example .env.local
```

3. Edit `.env.local` and add your Upstash credentials

4. Run development server:
```bash
npm run dev
```

5. Open [http://localhost:3000](http://localhost:3000)

---

## 🔧 Configuration

### Change Names

Edit `components/MoodBoard.tsx`:

```typescript
const [currentUser, setCurrentUser] = useState('YourName');
```

And update the title:
```jsx
<h1 className="title">YourName & PartnerName</h1>
```

### Add More Moods

Edit the `moodEmojis` array in `components/MoodBoard.tsx`:
```typescript
const moodEmojis = [
  { icon: Smile, label: 'happy', color: '#ffeb3b' },
  { icon: Heart, label: 'loved', color: '#ff6b9d' },
  // Add more here!
];
```

### Customize Styles

Edit `app/globals.css` to change colors, fonts, and layouts.

---

## 🎯 How It Works

### Data Flow

```
User Action → Frontend (React)
    ↓
API Route (/api/data)
    ↓
Upstash Redis (Cloud)
    ↓
Auto-sync every 5 seconds
    ↓
All devices updated!
```

### Real-time Sync

The app polls the API every 5 seconds to fetch new data. This means:
- Partner A adds a song → Partner B sees it within 5 seconds
- Both can be using the app simultaneously
- No conflicts or data loss

### AI Insights

When you click "AI Insights":
1. App sends mood/music data to Claude API
2. Claude analyzes patterns and compatibility
3. Returns personalized insights and suggestions
4. Displayed in a beautiful modal

---

## 📱 Mobile Support

The app is fully responsive and works great on:
- 📱 iPhone/Android phones
- 📱 Tablets
- 💻 Laptops
- 🖥️ Desktops

Camera access works on mobile for photo uploads!

---

## 🔐 Privacy & Security

- **Data Storage**: All data stored in Upstash Redis (encrypted)
- **Access Control**: Currently open (anyone with the URL can access)
- **AI Data**: Anonymized mood/music data sent to Anthropic API
- **Images**: Stored as base64 in Redis (not ideal for large scale)

### Want to Add Authentication?

You can add authentication to make the app private:
- Use **Clerk** (easiest)
- Use **NextAuth.js**
- Use **Supabase Auth**

This would give each couple their own private space.

---

## 🐛 Troubleshooting

### Build fails in Vercel

**Check:**
- Environment variables are set correctly
- `UPSTASH_REDIS_REST_URL` and `UPSTASH_REDIS_REST_TOKEN` are correct
- Node.js version is 18+

### Data not syncing

**Check:**
- Internet connection
- Upstash database is active
- Browser console for errors (F12)

### AI Insights not working

**Check:**
- You have some moods or songs added
- Internet connection
- Browser console for API errors

### Images too large

Images are stored as base64. For production:
- Use **Cloudinary** or **Uploadcare** for image hosting
- Store only URLs in Redis
- This is more scalable

---

## 🚀 Future Improvements

Want to make it even better?

- [ ] **Authentication** - Private couples spaces
- [ ] **Image hosting** - Use Cloudinary instead of base64
- [ ] **Notifications** - Push notifications for new entries
- [ ] **Spotify integration** - Play music directly
- [ ] **Export data** - Download as JSON/PDF
- [ ] **Mood streaks** - Track consecutive good mood days
- [ ] **Daily prompts** - AI-generated conversation starters
- [ ] **Multi-couple support** - Host for multiple couples

---

## 💰 Cost

- **Upstash Redis Free Tier**: 10,000 requests/day (plenty!)
- **Vercel Free Tier**: Unlimited deployments
- **Total**: $0/month for personal use 🎉

If you exceed free limits:
- Upstash: ~$0.20 per 100K requests
- Vercel: Still free for hobby projects

---

## 📄 License

MIT - Feel free to use and modify!

---

## 🙏 Credits

Built with love using:
- [Next.js](https://nextjs.org/)
- [Upstash](https://upstash.com/)
- [Vercel](https://vercel.com/)
- [Anthropic Claude](https://anthropic.com/)
- [Lucide Icons](https://lucide.dev/)

---

## 💬 Support

Having issues? Questions?
- Check the troubleshooting section above
- Open an issue on GitHub
- Check Vercel and Upstash docs

---

Made with ❤️ for couples everywhere
