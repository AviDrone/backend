# Linting and sorting using the python libraries black and isort

To push your code successfully through the GitHub Actions super-linter, you have to run the commands in a specific order.

To do so manually for all files in your current directory, use the following commands

```{bash}
black .
isort .
black .
```

Now, commit and push as you normally would with this message,  "Linted using black and isort".

To so lint and push automatically, run **bash scripts/git_lint_push.sh** from the **backend** directory.

```{bash}
chmod -x scripts/git_lint_push.sh
bash scripts/git_lint_push.sh
```

This first make an executable file and then does all the instructions above for you.
