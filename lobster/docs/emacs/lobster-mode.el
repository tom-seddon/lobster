;;; lobster-mode.el --- http://strlen.com/lobster

;; Author: Tom Seddon <lobster@tomseddon.plus.com>

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; lobster-mode is free software: you can redistribute it and/or
;; modify it under the terms of the GNU General Public License as
;; published by the Free Software Foundation, either version 3 of the
;; License, or (at your option) any later version.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Major mode for editing Lobster files. I quite like python.el, so I
;; copied bits of the indentation code (which looked like it would be
;; mostly impractical to reuse in situ) until I came up with this.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Indentation: Indentation level automatically increases after each
;; line that apparently opens a block (by ending with a colon). else:
;; is automatically dedented one stop. Repeated presses of TAB will
;; cycle through plausible indentation levels for the current line.

;; Indentation is by default 4 spaces.

;; Exec file: Press C-c C-c to invoke Lobster with the full path to
;; the current source file as its argument. Lobster's stdout/stderr
;; goes to the special specially-created *lobster TTY* buffer. (C-c
;; C-c will terminate the previously-invoked copy of Lobster, if there
;; is one.)

;; To configure the location of Lobster, set the variable
;; `lobster-program', a string; to specify additional arguments for
;; Lobster, set the variable `lobster-program-args', a list of
;; strings.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defcustom lobster-indent-offset 4
  "Default indentation offset for lobster."
  :group 'lobster
  :type 'integer
  :safe 'integerp)

(defcustom lobster-program "lobster"
  "Default program to use to run lobster files."
  :group 'lobster
  :type 'string
  :safe 'stringp)

(defcustom lobster-program-args '("-f" "-u")
  "Extra args to pass in to lobster program."
  :group 'lobster)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar lobster--indent-current-level 0
  "Index in `lobster--indent-levels' of indentation level `lobster-indent-line-function' is using.")

(defvar lobster--indent-levels '(0)
  "Levels of indentation available for `lobster-indent-line-function'.")

(defvar lobster--indent-dedenters-re (regexp-opt '("else:"))
  "Regexp matching words that should be dedented.")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--syntax-context (type ppss)
  (case type
    (comment
     (and (nth 4 ppss) (nth 8 ppss)))
    (string
     (and (not (nth 4 ppss)) (nth 8 ppss)))
    (paren
     (nth 1 ppss))
    (t
     nil)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster-nav-beginning-of-statement ()
  "Move to start of current statement."
  (interactive "^")
  (while (and (or (back-to-indentation) t)
              (not (bobp))
              (when (or
                     (save-excursion
                       (forward-line -1)
                       (python-info-line-ends-backslash-p))
                     (python-syntax-context 'string)
                     (python-syntax-context 'paren))
                (forward-line -1)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--indent-context ()
  "get indentation context info.

result is, like, \(STATUS . START)."
  (save-restriction
    (widen)
    (let (start
	  (ppss (save-excursion
		  (beginning-of-line)
		  (syntax-ppss))))
      (cond
       ;; First line?
       ((save-excursion
	  (goto-char (line-beginning-position))
	  (bobp))
	;; First line.
	(cons 'other nil))

       ;; Inside a paren?
       ((setq start (lobster--syntax-context 'paren ppss))
	(cons 'inside-paren start))
       
       ;; Inside a string?
       ((setq start (lobster--syntax-context 'string ppss))
	(cons 'inside-string start))

       ;; After beginning of block?
       ((setq start (save-excursion
		      (back-to-indentation)
		      (forward-comment -1)
		      (skip-syntax-backward "-")
		      (when (equal (char-before) ?:)
			(back-to-indentation)
			(point-marker))))
	(cons 'after-beginning-of-block start))

       ;; After normal line
       ((setq start (save-excursion
		      (back-to-indentation)
		      (forward-comment -1)
		      (lobster-nav-beginning-of-statement)
		      (point-marker)))
	(cons 'after-line start))
       
       ;; Bleargh
       (t
	(cons 'no-indent nil))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--indent-calculate-indentation ()
  "calculate appropriate indentation for current line."
  (interactive)
  (let* ((context (lobster--indent-context))
	 (status (car context))
	 (start (cdr context)))
    (save-restriction
      (widen)
      (save-excursion
	(case status
	  ('inside-string
	   ;; inside string - same indentation as previous
	   (goto-char start)
	   (current-indentation))

	  ('inside-paren
	   ;; inside nested construct - same indentation as opening.
	   (goto-char start)
	   (1+ (current-column)))
	  
	  ('after-beginning-of-block
	   ;; bump offset by one block's-worth.
	   (goto-char start)
	   (+ (current-indentation) lobster-indent-offset))

	  ('after-line
	   (let (result)
	     ;; assume previous's indentation
	     (save-excursion
	       (goto-char start)
	       (setq result (current-indentation)))

	     ;; if current line starts with a dedenter, dedent.
	     (back-to-indentation)
	     (when (looking-at lobster--indent-dedenters-re)
	       (setq result (max 0 (- result lobster-indent-offset))))

	     result))
	  
	  (t
	   ;; other... same indentation as previous.
	   (goto-char start)
	   (current-indentation)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--indent-calculate-levels ()
  (let* ((indentation (lobster--indent-calculate-indentation))
	 (steps (/ indentation lobster-indent-offset)))
    (setq lobster--indent-levels (list 0))

    ;; add tab stops
    (dotimes (step steps)
      (push (* lobster-indent-offset (1+ step)) lobster--indent-levels))

    ;; add current indent, if not aligned to indent granularity.
    (when (not (equal 0 (% indentation lobster-indent-offset)))
      (push indentation lobster--indent-levels))

    (setq lobster--indent-current-level (1- (length lobster--indent-levels)))
    (setq lobster--indent-levels (nreverse lobster--indent-levels)))
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--indent-cycle-levels ()
  (setq lobster--indent-current-level (1- lobster--indent-current-level))
  (when (< lobster--indent-current-level 0)
    (setq lobster--indent-current-level (1- (length lobster--indent-levels)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster-indent-line (&optional force-toggle)
  ;; if repeatedly pressing TAB, and there are multiple indent levels,
  ;; toggle. otherwise, recalculate.
  (if (or (and (eq this-command 'indent-for-tab-command)
	       (eq last-command this-command))
	  force-toggle)
      (if (not (equal lobster--indent-levels '(0)))
	  (lobster--indent-cycle-levels)
	(lobster--indent-calculate-levels))
    (lobster--indent-calculate-levels))

  ;; do it.
  (beginning-of-line)
  (delete-horizontal-space)
  (indent-to (nth lobster--indent-current-level lobster--indent-levels)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster-indent-line-function ()
  "`indent-line-function' for lobster-mode."
  (lobster-indent-line))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--syntax-comment-or-string-p ()
  "non-nil if point is inside comment or string."
  (nth 8 (syntax-ppss)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lobster--indent-electric-colon (arg)
  (interactive "*P")
  (self-insert-command (if (not (integerp arg))
			   1
			 arg))
  (message (format "%s %s" arg (eolp)))
  (when (and (not arg)			
	     (eolp)			
	     (not (equal ?: (char-after (- (point-marker) 2))))
	     (not (lobster--syntax-comment-or-string-p)))
    (let ((i (current-indentation))
	  (ci (lobster--indent-calculate-indentation)))
      (message (format "%d %d" i ci))
      (when (> i ci)
	(save-excursion
	  (indent-line-to ci))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-derived-mode lobster-tty-mode
  fundamental-mode "lobster-tty-mode"
  "major mode for lobster TTY output."

  ;; font lock muck.
  (setq font-lock-defaults
	`((,(rx line-start "---8<--- " (1+ anything) line-end)
	   (1 font-lock-string-face))
	  t				;keywords-only
	  nil				;case-fold
	  nil				;syntax-alist
	  nil				;syntax-begin
	  ))

  ;; compilation regexp.
  (set (make-local-variable 'compilation-error-regexp-alist)
       `(
	 ;; runtime error
	 (,(rx line-start (0+ space) (group (0+ not-newline)) "(" (group (1+ digit)) "): " (or "error" "VM error") ": " (0+ not-newline) line-end)
	  1				;file regexp group
	  2				;line regexp group
	  nil				;column regexp group
	  2 				;type (2=error)
	  )

	 ;; stack trace entry
	 (,(rx line-start (0+ space) "in block -> " (group (0+ not-newline)) "(" (group (1+ digit)) ")" line-end)
	  1				;file regexp group
	  2				;line regexp group
	  nil				;column regexp group
	  2				;type (2=error)
	  )

	 ;; post-run timing info
	 (,(rx line-start (0+ space) (group (0+ not-newline)) "(" (group (1+ digit)) "): " (1+ digit) "." (1+ digit) " %" line-end)
	  1				;file regexp group
	  2				;line regexp group
	  nil				;column regexp group
	  0				;type (0=info)
	  )
	 ))
  (compilation-shell-minor-mode 1)

  ;;
  (set (make-local-variable 'window-point-insertion-type) t)
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; the compilation error regexps don't seem to get processed on output
;; from a process. however, stuff inserted via `insert' does - so this
;; filter function just inserts the process's output using `insert'.
;;
;; see http://www.gnu.org/software/emacs/manual/html_node/elisp/Filter-Functions.html#Filter-Functions.
(defun lobster--exec-filter (proc string)
  (let ((buffer (process-buffer proc)))
    (when (buffer-live-p buffer)
      (with-current-buffer buffer
	(let* ((marker (process-mark proc))
	       (moving (= (point) marker)))
	  (save-excursion
	    (goto-char (process-mark proc))
	    (insert string)
	    (set-marker (process-mark proc) (point)))
	  (when moving
	    (goto-char marker)))))))

(defun lobster-exec-file ()
  (interactive)

  ;; prompt to save any modified lobster buffers.
  (save-some-buffers nil
		     (lambda ()
		       (eq major-mode 'lobster-mode)))
  
  (let* ((process-name "lobster-tty")
	 (buffer-name "*lobster TTY*")
	 (src-name (buffer-file-name)))

    ;; munge file name on windows.
    (if (eq system-type 'windows-nt)
	(setq src-name (replace-regexp-in-string "/" "\\\\" src-name)))
    
    ;; kill any existing lobster process.
    (let ((process (get-process process-name)))
      (when process
	(delete-process process)))

    ;; create new buffer, append separator, and run lobster process
    ;; with that as its buffer.
    (let* ((buffer (or (get-buffer buffer-name)
		       (with-current-buffer (generate-new-buffer buffer-name)
			 (lobster-tty-mode)
			 (current-buffer))))
	   (program-args `(,@lobster-program-args ,src-name)))
      ;; append separator
      (with-current-buffer buffer
	(goto-char (point-max))
	(unless (bolp)
	  (newline))
	(newline)
	;;(insert "")
	(newline)
	(insert "---8<--- "
		(format-time-string "%Y-%m-%d %R")
		" "
		(combine-and-quote-strings program-args))
	(newline)
	(compilation-forget-errors)
	(goto-char (point-max)))

      ;; start process
      (save-selected-window
	(let* ((w32-start-process-show-window t)
	       (process (apply 'start-process
			       process-name
			       buffer
			       lobster-program
			       program-args)))
	  (set-process-query-on-exit-flag process nil)
	  (set-process-filter process 'lobster--exec-filter)

	  (pop-to-buffer buffer)
	  
	  ;; this one DOES take effect.
	  (goto-char (point-max))
	  (set-marker (process-mark process) (point)))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar lobster-mode-syntax-table
  (let ((table (make-syntax-table)))
    ;; assign punctuation syntax to symbols or non-letter word
    ;; constituents. this is something python.el does and it's just the
    ;; ticket for dabbrev.
    (let ((symbol (string-to-syntax "_"))
	  (sst (standard-syntax-table)))
      (dotimes (i 128)
	(unless (= i ?_)
	  (when (equal symbol (aref sst i))
	    (modify-syntax-entry i "." table)))))

    ;; add C-style comments. see \\[c-populate-syntax-table].
    (modify-syntax-entry ?/ ". 124b" table)
    (modify-syntax-entry ?* " .23" table)
    (modify-syntax-entry ?\n "> b" table)

    ;; done...
    table))

(define-derived-mode lobster-mode
  prog-mode "lobster-mode"
  "major mode for editing lobster programs.

\\{lobster-mode-map}
Entry to this mode calls the value of
`lobster-mode-hook' if that value is non-nil."

  ;; comment
  (make-local-variable 'comment-start)
  (setq comment-start "//")

  ;; font lock.
  (setq font-lock-defaults '(nil nil nil))

  ;; indent line.
  (make-local-variable 'indent-line-function)
  (setq indent-line-function 'lobster-indent-line-function)
  (setq indent-tabs-mode nil)

  ;; imenu.
  (setq imenu-generic-expression
	;; `(1+ (syntax symbol))' didn't seem to do the trick for
	;; matching symbols.
	(let ((symbol '(seq (any "A-Za-z_") (0+ (any "A-Za-z_0-9"))))
	      (private '(?? (seq "private" (1+ space)))))
	  `(("function" ,(rx line-start (0+ space) (eval private) "function" (1+ space) (group (eval symbol)) (0+ space) "(") 1)
	    ("struct" ,(rx line-start (0+ space) (eval private) "struct" (1+ space) (group (eval symbol)) (0+ space) (any ":[")) 1)
	    ("value" ,(rx line-start (0+ space) (eval private) "value" (1+ space) (group (eval symbol)) (0+ space) (any ":[")) 1))))
  )

(define-key lobster-mode-map (kbd ":") 'lobster--indent-electric-colon)
(define-key lobster-mode-map (kbd "C-c C-c") 'lobster-exec-file)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgroup lobster nil
  "Major mode for editing lobster programs."
  :prefix "lobster-"
  :group 'languages)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar lobster-load-hook nil
  "hook for lobster-mode")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(run-hooks 'lobster-load-hook)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(provide 'lobster-mode)
