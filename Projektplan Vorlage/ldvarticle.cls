%%
%% This is file `ldvarticle.cls',
%% generated with the docstrip utility.
%%
%% The original source files were:
%%
%% ldvcommon.dtx  (with options: `article')
%% 
%% IMPORTANT NOTICE:
%% 
%% For the copyright see the source file.
%% 
%% Any modified versions of this file must be renamed
%% with new filenames distinct from ldvarticle.cls.
%% 
%% For distribution of the original source see the terms
%% for copying and modification in the file ldvcommon.dtx.
%% 
%% This generated file may be distributed as long as the
%% original source files, as listed above, are part of the
%% same distribution. (The sources need not necessarily be
%% in the same archive or directory.)
\NeedsTeXFormat{LaTeX2e}
\newcommand*\ldv@classversion{2008/09/23 Dokumentenklasse des LDV}
\ProvidesClass{ldvarticle}[\ldv@classversion]
\newcommand*\ldv@komaclass{scrartcl}
\RequirePackage{xkeyval}
\newcommand*\ldv@defaultinputenc{utf8}
\DeclareOptionX<ldv>{inputenc}[\ldv@defaultinputenc]{\def\ldv@inputenc{#1}}
\edef\@classoptionslist{english,\@classoptionslist}
\DeclareOptionX<ldv>{lang}[english]{%
  \XKV@addtolist@n\@classoptionslist{#1}%
}
\newcommand*\ldv@fontstylenr\@empty
\define@choicekey{ldv}{fontstyle}[\ldv@fontstyle\ldv@fontstylenr]
{serif,sans,helvet}{}
\newif\ifldv@isDivSet
\DeclareOptionX<ldv>{DIV}{\ldv@isDivSettrue}
\DeclareOptionX*{\PassOptionsToClass{\CurrentOption}{\ldv@komaclass}}
\ExecuteOptionsX<ldv>{%
  inputenc=\ldv@defaultinputenc,%
  fontstyle=helvet%
}
\ProcessOptionsX<ldv>\relax
\ifnum\ldv@fontstylenr=2%
  \ifldv@isDivSet\else
    \PassOptionsToClass{DIV=9}{\ldv@komaclass}%
    \ClassInfo{ldvklassen}{Set DIV to 9}%
  \fi
\fi
\newlength\ldv@laxLineWidth
\newlength\ldv@bcor  %% TODO
\newcount\ldv@div
\newcommand*\ldv@roundeddiv[2]{
  \newlength\ldv@tempdima
  \setlength\ldv@tempdima{#1}
  \divide\ldv@tempdima by#2
  \ldv@div\ldv@tempdima
  \ifnum \ldv@div<4
    \ldv@div=4
  \else
    \newlength\ldv@tempdimb
    \setlength\ldv@tempdimb{#1}%
    \divide\ldv@tempdimb by\ldv@div
    \addtolength\ldv@tempdima{\ldv@tempdima}%
    \addtolength\ldv@tempdima{-\ldv@tempdimb}%
    \advance\ldv@div by\@ne
    \setlength\ldv@tempdimb{#1}%
    \divide\ldv@tempdimb by\ldv@div
    \ifdim \ldv@tempdima<\ldv@tempdimb
      \advance\ldv@div by\m@ne
    \fi
  \fi
}
\newif\ifldv@mincl  %% TODO
\ldv@minclfalse
\newcommand*\ldv@getLaxLineWidth{%
  \settowidth\ldv@laxLineWidth{\normalfont%
eeeeeeeeeeeeeeeee nnnnnnnnnn iiiiiiii sssssss rrrrrrr aaaaaaa tttttt ddddd hhhhh uuuu lll ccc ggg mmm ooowwbbffkzv%
eeeeeeeeeeeee ttttttttt aaaaaaaa oooooooo iiiiiii nnnnnnn ssssss hhhhhh rrrrrr dddd llll ccc uuu mm ww ff gg yy pp bvk}%
  \setlength\ldv@laxLineWidth{.38\ldv@laxLineWidth}%
}
\newcommand*\ldv@getDIV{%
  \newlength\ldv@hBandWidth%
  \setlength\ldv@hBandWidth{\paperwidth}%
  \addtolength\ldv@hBandWidth{-\ldv@bcor}%
  \addtolength\ldv@hBandWidth{-\ldv@laxLineWidth}%
  \if@twocolumn%
    \addtolength\ldv@hBandWidth{-\columnsep}%
    \addtolength\ldv@hBandWidth{-\ldv@laxLineWidth}%
  \fi%
  \divide\ldv@hBandWidth by3%
  \ifldv@mincl%
    \setlength\ldv@hBandWidth{.75\ldv@hBandWidth}%
  \fi%
  \ifdim \ldv@hBandWidth <5mm%
    \setlength\ldv@hBandWidth{5mm}%
  \fi%
  \ldv@roundeddiv{\paperwidth}{\ldv@hBandWidth}%
  \ClassInfo{ldvklassen}{Computed DIV=\the\ldv@div.}%
}
\newcommand*\ldv@autotypearea{%
  \ldv@getLaxLineWidth%
  \ldv@getDIV%
  \PassOptionsToClass{DIV=\ldv@div}{ldv@komaclass}
}
%% Das ist hier noch nicht möglich, weil die Schriftgröße noch nicht
%% eingestellt ist.
%% \ldv@autotypearea
\LoadClass{\ldv@komaclass}
\RequirePackage[\ldv@inputenc]{inputenc}
\RequirePackage[T1]{fontenc}
\RequirePackage{babel}
\RequirePackage{array}
\RequirePackage{fancyvrb}
\fvset{xleftmargin=2em}
\newcommand\emphemph[1]{{\bfseries#1}}
\newcommand\simpleverb[1]{{\ttfamily#1}}
\newcommand*\notename{Note}
\addto\captionsngerman{%
  \renewcommand*\notename{Bemerkung}%
}
\addto\captionsgerman{%
  \renewcommand*\notename{Bemerkung}%
}
\newenvironment{note}[1][\notename]{%
  \vspace{\baselineskip}%
  \noindent\small{\usekomafont{sectioning}#1:}\hspace{.5em}%
}{}
\RequirePackage{color}
\RequirePackage{graphicx}
\newlength\graphicswidth
\setlength\graphicswidth{\textwidth}
\addtolength\graphicswidth{-2em}
\newlength\graphicswidthtwo
\setlength\graphicswidthtwo{.5\textwidth}
\addtolength\graphicswidthtwo{-3em}
\setcapindent{0pt}
\setkomafont{caption}{\small}
\setkomafont{captionlabel}{\usekomafont{sectioning}}
\definecolor{TUMBlau}       {cmyk}{1.00,0.43,0.00,0.00}
\definecolor{TUMBlau1}      {cmyk}{1.00,0.57,0.12,0.70}
\definecolor{TUMBlau2}      {cmyk}{1.00,0.54,0.04,0.19}
\definecolor{TUMBlau3}      {cmyk}{0.90,0.48,0.00,0.00}
\definecolor{TUMBlau4}      {cmyk}{0.65,0.19,0.01,0.04}
\definecolor{TUMBlau5}      {cmyk}{0.42,0.09,0.00,0.00}
\definecolor{TUMDunkelgrau} {cmyk}{0.00,0.00,0.00,0.80}
\definecolor{TUMMittelgrau} {cmyk}{0.00,0.00,0.00,0.50}
\definecolor{TUMHellgrau}   {cmyk}{0.00,0.00,0.00,0.20}
\definecolor{TUMGruen}      {cmyk}{0.35,0.00,1.00,0.20}
\definecolor{TUMOrange}     {cmyk}{0.00,0.65,0.95,0.00}
\definecolor{TUMElfenbein}  {cmyk}{0.03,0.04,0.14,0.08}
\newcommand*\ldv@setfontstyle{%
  \ifcase\ldv@fontstylenr\relax%
    \renewcommand\familydefault{\rmdefault}%
    % The following font definitions are taken from the KOMA-Script
    % classes. Only the '\sffamily' command is left out.
  \or%
    \renewcommand\familydefault{\sfdefault}%
    % disposition, descriptionlabel and dictum fonts already are
    % \sffamily in KOMA-Script
  \or%
    \RequirePackage{mathptmx}%
    \RequirePackage[scaled=0.92]{helvet}%
    \renewcommand\familydefault{\sfdefault}%
    % disposition, descriptionlabel and dictum fonts already are
    % \sffamily in KOMA-Script
  \fi%
  \setkomafont{disposition}{\normalcolor\bfseries}%
  \setkomafont{descriptionlabel}{\normalcolor\itshape}%
  \setkomafont{dictum}{\normalfont\normalcolor\small}%
}
\ldv@setfontstyle
\tolerance=1414
\hbadness=1414
\emergencystretch=1.5em
\hfuzz=0.3pt
\widowpenalty=10000
\vfuzz\hfuzz
\raggedbottom
\RequirePackage{amsmath}
\RequirePackage{amssymb}
\newtheorem{definition}{Definition}
\newcommand*\theoremname{Theorem}
\addto\captionsngerman{%
  \renewcommand*\theoremname{Satz}%
}
\addto\captionsgerman{%
  \renewcommand*\theoremname{Satz}%
}
\newtheorem{theorem}{\theoremname}
\providecommand*\doi[1]{\href{http://dx.doi.org/#1}{doi:#1}}
\RequirePackage{natbib}
\let\ldv@latex@bibliographystyle=\bibliographystyle
\renewcommand*\bibliographystyle[1]{%
  \ldv@latex@bibliographystyle{#1}\ldv@bibstylesettrue%
}
\newif\ifldv@bibstyleset
\let\ldv@latex@bibliography=\bibliography
\renewcommand*\bibliography[1]{%
  \ifldv@bibstyleset\else\bibliographystyle{plainnat}\fi%
  \ldv@latex@bibliography{#1}%
}
\renewcommand*\cite{\citep}
\RequirePackage{nameref}
\RequirePackage{varioref}
\RequirePackage[pdfborder={0 0 0},anchorcolor=blue]{hyperref}
\newcommand*{\@keywords}{}
\newcommand{\keywords}[1]{%
  \renewcommand*\@keywords{#1}%
  \hypersetup{pdfkeywords = {#1}}%
}
\newcommand*\keywordname{Key words}%
\addto\captionsngerman{%
  \renewcommand*\keywordname{Schl\"usselw\"orter}%
}
\addto\captionsgerman{%
  \renewcommand*\keywordname{Schl\"usselw\"orter}%
}
\newcommand{\makekeywords}{%
  \if@twocolumn
  \vspace{\topsep}
  \noindent{\sectfont\size@paragraph\keywordname:} \@keywords
  \else
  \small
  \begin{center}%
    {\normalfont\sectfont\nobreak\keywordname
      \vspace{-.5em}\vspace{\z@}}%
  \end{center}%
  \quotation\@keywords\endquotation
  \fi}
\newcommand*{\@version}{}
\newcommand{\version}[1]{\renewcommand*\@version{#1}}
\publishers{Technische Universit\"at M\"unchen, LDV}
\renewcommand*\title[1]{%
  \renewcommand*\@title{#1}%
  \hypersetup{%
    pdftitle = {\@title\ifx\@subtitle\@empty\else. \@subtitle.\fi}%
  }%
}
\renewcommand*\subtitle[1]{%
  \renewcommand*\@subtitle{#1}%
  \hypersetup{%
    pdftitle = {\ifx\@title\@empty\else\@title. \@subtitle.\fi}%
  }%
}
\renewcommand*\author[1]{%
  \renewcommand*\@author{#1}%
  \let\tempand=\and%
  \renewcommand*\and{, }%
  \let\tempthanks=\thanks%
  \let\thanks=\@empty%
  \hypersetup{%
    pdfauthor  = {\textcopyright\ \@author}%
  }%
  \let\and=\tempand%
  \let\thanks=\tempthanks%
}
\renewcommand*\subject[1]{%
  \renewcommand*\@subject{#1}%
  \hypersetup{%
    pdfsubject = {\@subject}
  }%
}
\RequirePackage{calc}
\renewcommand*\subject@font{\normalfont\normalcolor\bfseries}
\newlength\ldv@ldvlogowidth
\setlength\ldv@ldvlogowidth{19mm}
\newcommand*\ldv@ldvlogoS{LDVLogoS_oT}
\newcommand*\ldv@ldvlogoCMYK{LDVLogoCMYK_oT}
\newlength\ldv@tumlogowidth
\setlength\ldv@tumlogowidth{17mm}
\newcommand*\ldv@tumlogoS{TUMLogo_oZ_Vollfl_sw}
\newcommand*\ldv@tumlogoCMYK{TUMLogo_oZ_Vollfl_CMYK}
\define@choicekey*{ldv}{frontcover}[\ldv@frontcoverdesign]
  {design1}{%
    \if@titlepage\else%
      \ClassWarning{\ClassName}{%
        Option frontcover of \string\maketitle\ is only valid
        \MessageBreak
        when using titlepage=true%
      }%
    \fi%
  }%
\define@key{ldv}{pagenumber}{%
  \if@titlepage%
    \setcounter{page}{#1}%
  \else%
    \ClassWarning{\ClassName}{%
      Option pagenumber of \string\maketitle\ is only valid
      \MessageBreak
      when using titlepage=true%
    }%
  \fi%
}
\global\let\@maketitle\relax
\if@titlepage
  \renewcommand*\maketitle[1][]{%
    \setkeys{ldv}{#1}%
    \begin{titlepage}
      \let\footnotesize\small
      \let\footnoterule\relax
      \let\footnote\thanks
      \renewcommand*\thefootnote{\@fnsymbol\c@footnote}%
      \let\@oldmakefnmark\@makefnmark
      \renewcommand*{\@makefnmark}{\rlap\@oldmakefnmark}%
      \ifx\ldv@frontcoverdesign\undefined\else%
        \ifstr{\ldv@frontcoverdesign}{design1}{%
          \setcounter{page}{-1}%
          \ldv@makecover@eins%
          \if@twoside\cleardoubleemptypage\else\clearpage\fi%
          \thispagestyle{empty}
        }{}%
      \fi%
      \ifx\@extratitle\@empty \else
        \noindent\@extratitle\next@tpage\cleardoubleemptypage
        \thispagestyle{empty}%
      \fi
      \ldv@pagetitle
      \if@twoside\next@tpage
        \begin{minipage}[t]{\textwidth}
          \@uppertitleback
        \end{minipage}\par
        \vfill
        \begin{minipage}[b]{\textwidth}
          \@lowertitleback
        \end{minipage}
      \fi
      \ifx\@dedication\@empty \else
        \next@tpage\null\vfill
        {\centering \Large \@dedication \par}%
        \vskip \z@ \@plus3fill
        \if@twoside \next@tpage\cleardoubleemptypage \fi
      \fi
    \end{titlepage}
    \setcounter{footnote}{0}%
    \global\let\thanks\relax
    \global\let\maketitle\relax
    \global\let\@thanks\@empty
  }
\else
  \renewcommand*\maketitle[1][]{\par%
    \setkeys{ldv}{#1}%
    \begingroup
      \renewcommand*\thefootnote{\@fnsymbol\c@footnote}%
      \let\@oldmakefnmark\@makefnmark
      \renewcommand*{\@makefnmark}{\rlap\@oldmakefnmark}
      \if@twocolumn
        \ifnum \col@number=\@ne
          \ldv@headtitle
         \else
          \twocolumn[\ldv@headtitle]%
        \fi
      \else
        \newpage
        \global\@topnum\z@
        \ldv@headtitle
      \fi
      \thispagestyle{\titlepagestyle}\@thanks
    \endgroup
    \setcounter{footnote}{0}%
    \let\thanks\relax
    \let\maketitle\relax
    \global\let\@thanks\@empty
  }
\fi
\if@titlepage
\newcommand*\ldv@pagetitle{%
  \ldv@makepublishers%
  \setparsizes{\z@}{\z@}{\z@\@plus 1fil}\par@updaterelative%
  \ifx\@titlehead\@empty \else%
  \begin{minipage}[t]{\textwidth}%
    \@titlehead
  \end{minipage}\par
  \fi
  \null\vfill
  \begin{center}
    \ifx\@subject\@empty\else
    {\subject@font \@subject \par}%
    \vskip 3em
    \fi
    {\titlefont\huge \@title\par}%
    \ifx\@subtitle\@empty\else%
      \vskip 1em%
      {\usekomafont{subtitle}\@subtitle\par}\fi%
    \vskip 3em
    {\lineskip 0.75em%
      \ifx\ldv@studthesispersons\undefined%
      \begin{tabular}[t]{c}
        \@author
      \end{tabular}\par%
      \else%
      \ldv@studthesispersons\par%
      \fi
    }%
    \vskip 1.5em
    {\@date%
      \ifx\@version\@empty\else, \@version\fi\par}%
    \vfill\vfill\vfill\vfill\null
  \end{center}\par
  \@thanks
}
\newcommand*\ldv@makepublishers{%
  \newlength\ldv@tumwidth%
  \newlength\ldv@ldvwidth%
  \newlength\ldv@publisherwidth%
  \settowidth\ldv@tumwidth{Technische Universit\"at M\"unchen}%
  \settowidth\ldv@ldvwidth{Lehrstuhl f\"ur Datenverarbeitung}%
  \ifdim\ldv@tumwidth >\ldv@ldvwidth%
    \setlength\ldv@publisherwidth{\ldv@tumwidth}%
  \else%
    \setlength\ldv@publisherwidth{\ldv@ldvwidth}%
  \fi%
  \newsavebox\ldv@publishersbox%
  \savebox\ldv@publishersbox[\textwidth]{%
    \parbox{\ldv@ldvlogowidth}{%
      \includegraphics[width=\ldv@ldvlogowidth]{\ldv@ldvlogoS}%
    }%
    \hfill%
    \parbox{\ldv@publisherwidth}{%
      \centering%
      Lehrstuhl f\"ur Datenverarbeitung\\
      Technische Universit\"at M\"unchen%
    }%
    \hfill%
    \parbox{\ldv@tumlogowidth}{%
      \includegraphics[width=\ldv@tumlogowidth]{\ldv@tumlogoS}%
    }%
  }%
  %
  \newlength\ldv@bottommargin
  \setlength\ldv@bottommargin{1in + \hoffset + \oddsidemargin}
  \newlength\ldv@publishersboxdepth%
  \settodepth\ldv@publishersboxdepth{\usebox\ldv@publishersbox}%
  \newlength\ldv@publishersbottom%
  \setlength\ldv@publishersbottom{%
    1in + \voffset + \topmargin + \headheight + \headsep +%
    \topskip + \ldv@publishersboxdepth%
  }%
  \newlength\ldv@publishersmove%
  \setlength\ldv@publishersmove{%
    \paperheight - \ldv@publishersbottom - \ldv@bottommargin%
  }%
  \noindent\raisebox{-\ldv@publishersmove}[0pt][0pt]{%
    \makebox[0pt][l]{{\usebox\ldv@publishersbox}}}%
}
\fi
\if@titlepage\else
\newcommand*\ldv@headtitle{%
  \clearpage
  \let\footnote\thanks%
  \ifx\@extratitle\@empty \else
    \noindent\@extratitle \next@tpage \if@twoside \null\next@tpage \fi
  \fi
  \noindent\includegraphics[width=\ldv@ldvlogowidth]{\ldv@ldvlogoCMYK}%
  \hfill%
  \includegraphics[width=\ldv@tumlogowidth]{\ldv@tumlogoCMYK}\par%
  \setparsizes{\z@}{\z@}{\z@\@plus 1fil}\par@updaterelative
  \vskip .5em
  \ifx\@titlehead\@empty \else
    \begin{minipage}[t]{\textwidth}
      \@titlehead
    \end{minipage}\par
  \fi
  \null
  \vskip 1.5em%
  \begin{center}%
    \ifx\@subject\@empty \else
      {\subject@font \@subject \par}
      \vskip 1.5em
    \fi
    {\titlefont\huge \@title \par}%
    \ifx\@subtitle\@empty\else%
      \vskip .5em%
      {\usekomafont{subtitle}\@subtitle\par}\fi%
    \vskip 1.5em
    {\lineskip .5em%
      \begin{tabular}[t]{c}
        \@author
      \end{tabular}\par
    }%
    \vskip 1em%
    {\@date%
      \ifx\@version\@empty\else, \@version\fi \par}%
    \ifx\@dedication\@empty \else
      \vskip 2em
      {\Large \@dedication \par}
    \fi
  \end{center}%
  \par
  \vskip 2em
}
\fi
\newcommand*\@covertitle{}
\newcommand*\covertitle[1]{%
  \renewcommand*\@covertitle{#1}%
}
\if@titlepage
\newcommand\ldv@makecover@eins{%
  \newlength\ldv@coverwidth%
  \setlength\ldv@coverwidth{\paperwidth - \hoffset}%
  \newlength\ldv@coverheight%
  \setlength\ldv@coverheight{\paperheight - \voffset}%
  \newlength\ldv@coverleftcolumnwidth%
  \setlength\ldv@coverleftcolumnwidth{0.618\ldv@coverwidth - 4mm}%
  \newlength\ldv@coverrightcolumnwidth%
  \setlength\ldv@coverrightcolumnwidth{%
    \ldv@coverwidth - \ldv@coverleftcolumnwidth - 8mm}%
  \newsavebox\ldv@covertitleauthor%
  \savebox\ldv@covertitleauthor{%
    \parbox{\ldv@coverwidth}{%
      \hspace{0.1\ldv@coverwidth}%
      \parbox{\ldv@coverleftcolumnwidth - 0.1\ldv@coverwidth}{%
        \raggedleft\bfseries%
        {\huge\ifx\@covertitle\empty\@title\else\@covertitle\fi}\\
        \@subtitle%
      }%

      \vspace{1mm}\hspace{\ldv@coverleftcolumnwidth}\hspace{8mm}%
      \parbox{\ldv@coverrightcolumnwidth - 0.05\ldv@coverwidth}{%
        \let\tempand=\and%
        \let\and=\newline%
        \renewcommand*\thanks[1]{}%
        \raggedright\bfseries%
        \@author%
        \let\and=\tempand%
      }%
    }%
  }%
  \newsavebox\ldv@coverlogos%
  \savebox\ldv@coverlogos[0mm][l]{%
    \parbox[c]{\ldv@coverleftcolumnwidth}{%
      \raggedleft%
      \includegraphics[width=\ldv@ldvlogowidth]{\ldv@ldvlogoCMYK}%
    }%
    \hspace{8mm}%
    \parbox[c]{\ldv@tumlogowidth}{%
      \raggedright%
      \includegraphics[width=\ldv@tumlogowidth]{\ldv@tumlogoCMYK}%
    }%
  }%
  %\setlength\fboxsep{0mm}%
  \noindent\hspace{-1in}\hspace{-\oddsidemargin}%
  \raisebox{1in + \topmargin + \headheight + \headsep +
    \topskip}[0mm][0mm]{%
    \parbox[t][\ldv@coverheight]{\ldv@coverwidth}{%
      \newlength\ldv@coveremptyspace%
      \setlength\ldv@coveremptyspace{%
        \ldv@coverheight
        - \totalheightof{\usebox\ldv@covertitleauthor}
        - \totalheightof{\usebox\ldv@coverlogos}
      }%
      \raisebox{-\ldv@coverheight}[0mm][0mm]{\makebox[0mm][l]{%
          \hspace{\ldv@coverleftcolumnwidth}\hspace{2.6mm}%
          \color{TUMBlau5}\rule{3mm}{\ldv@coverheight}%
        }%
      }%
      \raisebox{-\ldv@coverheight + 0.095\ldv@coveremptyspace + \totalheight}[0mm][0mm]{%
        \usebox\ldv@coverlogos%
      }%
      \raisebox{-\height - 0.25\ldv@coveremptyspace}{%
        \usebox\ldv@covertitleauthor%
      }%
    }%
  }%
}
\fi
\endinput
%%
%% End of file `ldvarticle.cls'.
