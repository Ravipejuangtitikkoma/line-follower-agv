# Panduan Perintah Harian Git Kolaborasi

Daftar perintah Git yang digunakan sehari-hari saat bekerja sama dalam satu repositori bersama tim.

---

## 1. Sebelum Mulai Ngoding (Ambil Update & Buat Branch Baru)
Selalu tarik pembaruan terbaru dari `main` terlebih dahulu sebelum membuat branch untuk tugas baru agar tidak bentrok dengan kode teman.

```bash
git checkout main
git pull origin main
git checkout -b nama-tugas
```

> **Catatan:** Jika masih melanjutkan pekerjaan di branch yang sudah dibuat sebelumnya, cukup pindah ke branch tersebut lalu tarik update dari `main`:
> ```bash
> git checkout nama-tugas
> git pull origin main
> ```

---

## 2. Simpan & Upload Perubahan (Setelah Edit Kode)
Setelah selesai mengedit dan menyimpan (*Save*) file di editor, cek status lalu simpan perubahannya ke Git.

```bash
git status
git add .
git commit -m "feat: jelaskan perubahan yang kamu buat"
```

### Upload (Push) ke GitHub
* **Push pertama kali pada branch baru:**
  ```bash
  git push -u origin nama-tugas
  ```
* **Push kedua dan seterusnya pada branch yang sama:**
  ```bash
  git push
  ```

---

## 3. Gabungkan Kode ke `main` (Pull Request)
1. Buka link Pull Request yang muncul di terminal setelah `git push`, atau buka tab **Pull requests** di halaman GitHub repositori.
2. Klik **Compare & pull request** → tulis keterangan singkat → klik **Create pull request**.
3. Tunggu *review* dari rekan tim, lalu klik **Merge pull request**.

---

## 4. Setelah Pull Request Di-merge (Sinkronisasi & Bersih-bersih)
Tarik hasil gabungan terbaru ke laptop dan hapus branch lama yang sudah tidak terpakai agar repositori lokal tetap rapi.

```bash
git checkout main
git pull origin main
git branch -d nama-tugas
```

---

## 5. Perintah Bantu yang Sering Dipakai

| Perintah | Fungsi |
| :--- | :--- |
| `git branch` | Melihat daftar branch lokal dan posisi branch aktif saat ini |
| `git checkout nama-branch` | Pindah ke branch lain yang sudah ada |
| `git log --oneline -n 5` | Melihat 5 riwayat commit terakhir secara ringkas |
| `git push origin --delete nama-branch` | Menghapus branch yang ada di GitHub |

---

## Aturan Singkat Kolaborasi
* **Jangan push langsung ke `main`** — selalu kerjakan perubahan di branch terpisah.
* **Wajib `git pull`** sebelum mulai mengedit kode untuk meminimalkan *merge conflict*.
* **Gunakan pesan commit yang jelas** — contoh awalan: `feat:` (fitur baru), `fix:` (perbaikan bug), `chore:` (rapih-rapih/konfigurasi), `docs:` (dokumentasi).
* **Hindari `git push --force`** pada branch yang dipakai bersama.