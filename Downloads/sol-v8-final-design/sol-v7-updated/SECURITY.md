# Security Guidelines 🔐

## Important: Protecting Your Passwords

This app has been configured to keep your passwords secure and out of version control.

## Configuration Files

### Passwords: `src/config.js`
- **Status**: ✅ Gitignored (will NOT be committed)
- **Contains**: User passwords
- **Setup**: Copy `src/config.example.js` to `src/config.js` and update

### API Keys: `.env`
- **Status**: ✅ Gitignored (will NOT be committed)
- **Contains**: Spotify API credentials (optional)
- **Setup**: Copy `.env.example` to `.env` and update

## Files That Are Safe to Commit

✅ `src/config.example.js` - Template without real passwords
✅ `.env.example` - Template without real API keys
✅ `.gitignore` - List of files to ignore
✅ All other source code files

## Files That Should NEVER Be Committed

❌ `src/config.js` - Contains real passwords
❌ `.env` - Contains real API keys
❌ `node_modules/` - Dependencies
❌ `.vercel/` - Deployment config

## Before First Commit

Before pushing to GitHub for the first time:

```bash
# 1. Create your config.js
cp src/config.example.js src/config.js
# Edit config.js with your passwords

# 2. Verify gitignore is working
git status

# You should NOT see:
# - src/config.js
# - .env
# - node_modules/

# 3. If you see config.js, check your .gitignore
cat .gitignore | grep config.js
```

## If You Accidentally Committed Passwords

If you accidentally committed `config.js` or `.env`:

### Option 1: Remove from latest commit (if not pushed)
```bash
git rm --cached src/config.js
git rm --cached .env
git commit --amend -m "Remove sensitive files"
```

### Option 2: Remove from history (if already pushed)
```bash
# This is more complex - contact repo admin or:
git filter-branch --force --index-filter \
  "git rm --cached --ignore-unmatch src/config.js" \
  --prune-empty --tag-name-filter cat -- --all

git push origin --force --all
```

### Option 3: Change your passwords
If passwords were exposed:
1. Change passwords in `src/config.js`
2. Deploy new version
3. Notify users of new passwords

## Deployment Security

### Vercel Environment Variables
For production deployment, you can also use Vercel environment variables instead of `.env`:

1. Go to Vercel Dashboard
2. Select your project
3. Go to Settings → Environment Variables
4. Add:
   - `SPOTIFY_CLIENT_ID`
   - `SPOTIFY_CLIENT_SECRET`
   - `SPOTIFY_REDIRECT_URI`

### Password Security
- Passwords are stored client-side only
- Use strong, unique passwords
- Don't share passwords in public channels
- Change passwords if they're compromised

## Checking Your Security

Run this checklist before deploying:

- [ ] `config.js` is NOT in git status
- [ ] `.env` is NOT in git status
- [ ] `.gitignore` exists and includes both files
- [ ] Passwords in `config.js` are strong
- [ ] No hardcoded passwords in `App.jsx`
- [ ] Example files (`.example.js`, `.env.example`) don't have real credentials

## Questions?

**Q: Can I commit config.example.js?**
A: Yes! It only contains placeholders, not real passwords.

**Q: How do collaborators get the passwords?**
A: Share them through a secure channel (encrypted message, password manager), NOT through Git.

**Q: What if I lose my config.js?**
A: You'll need to recreate it from `config.example.js` and set new passwords.

**Q: Can I use different passwords for dev and production?**
A: Yes! Keep separate `config.js` files for each environment (don't commit either).

---

**Remember**: Security is important! Keep your passwords safe. 🔒
