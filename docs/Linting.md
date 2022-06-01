# Linting and sorting

Using black and isort.

To make the GitHub Actions pipeline, you have to run the commands in a specific order.

To do so manually for all files in your current directory, use the following commands

```{bash}
isort .
black .
```

To so automatically, run **bash scripts/git_lint_push.sh**

```{bash}
chmod -x scripts/git_lint_push.sh
bash scripts/git_lint_push.sh
```