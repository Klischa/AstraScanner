# GitHub PR Reviewer Automation

Cron-автоматизация OpenHands: следит за pull request'ами в репозитории
и запускает AI-ревью, когда на PR навешивают метку-триггер.

## Параметры (main.py, константы вверху файла)

| Константа | Значение |
|---|---|
| `REPOS` | `Klischa/AstraScanner` |
| `TRIGGER_LABEL` | `openhands-review` |
| `REVIEW_TONE` | `thorough` |
| `REPO_REVIEW_GUIDE_PATH` | `.agents/skills/custom-codereview-guide.md` (подключается если файл существует) |

## Как использовать

1. Открой pull request;
2. Навесь метку `openhands-review` — автоматизация (в течение интервала опроса, по умолчанию 5 мин) запустит ревью и опубликует его как **pull request review**;
3. Каждая метка обрабатывается один раз. Чтобы запросить новое ревью — сними метку и навесь заново.



## Требования

- Секрет `GITHUB_PERSONAL_ACCESS_TOKEN` в OpenHands Settings → Secrets (Pull requests: Read and Write; Contents: Read; Issues: Read and Write; Metadata: Read).
- Облачный automation backend OpenHands.

## Исходники

`main.py` — официальный скрипт скилла
[`github-pr-reviewer`](https://github.com/OpenHands/extensions/tree/main/skills/github-pr-reviewer)
(с подставленной константой `REPOS`).