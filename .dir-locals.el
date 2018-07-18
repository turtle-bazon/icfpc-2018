((nil (indent-tabs-mode . nil)
      (tab-width . 2)
      (fill-column . 80)
      (c-basic-offset . 2)
      (mode . npm)
      (eval ignore-errors
            "Write-contents-functions is a buffer-local alternative to before-save-hook"
            (add-hook 'before-save-hook 'delete-trailing-whitespace nil 'make-it-local)))
 (json-mode . ((js-indent-level . 2)))
 (javascript-mode . ((js-indent-level . 2)))
 (js2-mode . ((js-indent-level . 2)))
 (typescript-mode . ((typescript-indent-level . 2)))
 (sql-mode . ((sql-product . postgres))))
