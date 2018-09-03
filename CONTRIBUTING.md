# Contributing to OkapiLib

:+1::tada: First off, thanks for taking the time to contribute! :tada::+1:

The following is a set of guidelines for contributing to OkapiLib. These are mostly guidelines, not rules. Use your best judgment, and feel free to propose changes to this document in a pull request.

#### Table Of Contents

[Code of Conduct](#code-of-conduct)

[I don't want to read this whole thing, I just have a question!!!](#i-dont-want-to-read-this-whole-thing-i-just-have-a-question)

[How Can I Contribute?](#how-can-i-contribute)
  * [Reporting Bugs](#reporting-bugs)
  * [Suggesting Enhancements](#suggesting-enhancements)
  * [Your First Code Contribution](#your-first-code-contribution)
  * [Pull Requests](#pull-requests)

[Styleguides](#styleguides)
  * [Git Commit Messages](#git-commit-messages)
  * [Git Branch Naming](#git-branch-naming)
  * [Code Styleguide](#code-styleguide)

## Code of Conduct

This project and everyone participating in it is governed by OkapiLib's [Code of Conduct](code-of-conduct.md). By participating, you are expected to uphold this code.

## I don't want to read this whole thing I just have a question!!!

You can get help for questions that can't be answered by [OkapiLib's docs](https://pros.cs.purdue.edu/v5/okapi/index.html) by [filing an issue](https://github.com/OkapiLib/OkapiLib/issues/new/choose).

## How Can I Contribute?

### Reporting Bugs

This section guides you through submitting a bug report. Following these guidelines helps maintainers and the community understand your report :pencil:, reproduce the behavior :computer: :computer:, and find related reports :mag_right:.

Before creating bug reports, please check [this list](#before-submitting-a-bug-report) as you might find out that you don't need to create one. When you are creating a bug report, please [include as many details as possible](#how-do-i-submit-a-good-bug-report). Fill out [the required template](ISSUE_TEMPLATE.md), the information it asks for helps us resolve issues faster.

> **Note:** If you find a **closed** issue that seems like it is the same thing that you're experiencing, open a new issue and include a link to the original issue in the body of your new one.

#### Before Submitting A Bug Report

* **Perform a cursory search** to see if the problem has already been reported. If it has **and the issue is still open**, add a comment to the existing issue instead of opening a new one.

#### How Do I Submit A (Good) Bug Report?

Bugs are tracked as [GitHub issues](https://guides.github.com/features/issues/). Create an issue and provide the following information by filling in [the template](ISSUE_TEMPLATE.md).

Explain the problem and include additional details to help maintainers reproduce the problem:

* **Use a clear and descriptive title** for the issue to identify the problem.
* **Describe the exact steps which reproduce the problem** in as many details as possible. When listing steps, **don't just say what you did, but explain how you did it**. For example, if you used a `ChassisController` to drive the robot forward, don't just explain the arguments, but also explain how the method you are calling is used in your code.
* **Provide specific examples to demonstrate the steps**. Include links to files or GitHub projects, or copy/pasteable snippets, which you use in those examples. If you're providing snippets in the issue, use [Markdown code blocks](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#code).
* **Describe the behavior you observed after following the steps** and point out what exactly is the problem with that behavior.
* **Explain which behavior you expected to see instead and why.**
* **If the problem wasn't triggered by a specific action**, describe what you were doing before the problem happened and share more information using the guidelines below.

Provide more context by answering these questions:

* **Did the problem start happening recently** (e.g. after updating to a new version) or was this always a problem?
* If the problem started happening recently, what's the most recent version in which the problem doesn't happen?
* **Can you reliably reproduce the issue?** If not, provide details about how often the problem happens and under which conditions it normally happens.

Include details about your configuration and environment:

* **Which version of OkapiLib and PROS are you using?** You can get the version by running `pros conduct info-project` in your terminal.

### Suggesting Enhancements

This section guides you through submitting an enhancement suggestion, including completely new features and minor improvements to existing functionality. Following these guidelines helps maintainers and the community understand your suggestion :pencil: and find related suggestions :mag_right:.

Before creating enhancement suggestions, please check [this list](#before-submitting-an-enhancement-suggestion) as you might find out that you don't need to create one. When you are creating an enhancement suggestion, please [include as many details as possible](#how-do-i-submit-a-good-enhancement-suggestion). Fill in [the template](ISSUE_TEMPLATE.md), including the steps that you imagine you would take if the feature you're requesting existed.

#### Before Submitting An Enhancement Suggestion

* **Perform a cursory search** to see if the enhancement has already been suggested. If it has, add a comment to the existing issue instead of opening a new one.

#### How Do I Submit A (Good) Enhancement Suggestion?

Enhancement suggestions are tracked as [GitHub issues](https://guides.github.com/features/issues/). Create an issue on that repository and provide the following information:

* **Use a clear and descriptive title** for the issue to identify the suggestion.
* **Provide a step-by-step description of the suggested enhancement** in as many details as possible.
* **Provide specific examples to demonstrate the steps**. Include copy/pasteable snippets which you use in those examples, as [Markdown code blocks](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#code).
* **Explain why this enhancement would be useful** to most users.
* **Specify which version of OkapiLib and PROS you're using.** You can get the version by running `pros conduct info-project` in your terminal.

### Your First Code Contribution

Unsure where to begin contributing? You can start by looking through these `beginner` and `help-wanted` issues:

* [Beginner issues][good first issue] - issues which should only require a few lines of code
* [Help wanted issues][help wanted] - issues which should be a bit more involved than `beginner` issues.

### Pull Requests

* Fill in [the required template](PULL_REQUEST_TEMPLATE.md)
* Title the pull request [issue number][description]
 * For example, `Issue #0: Add a readme file`
* End all files with a newline
* Follow the [style guide](#code-styleguide)

## Styleguides

### Git Commit Messages

* Use the present tense ("Add feature" not "Added feature")
* Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
* Title the commit message [issue number][message]
  * For example, `Issue #0: Initial commit adding a readme file`
* Place additional information after the first line

### Git Branch Naming

* This project uses Git flow. If you are not familiar, look here: http://nvie.com/posts/a-successful-git-branching-model/
  * If Git flow looks difficult or confusing, you can download [GitKraken](https://www.gitkraken.com/) and it will handle branching/merging for you ([tutorial here](https://support.gitkraken.com/git-workflows-and-extensions/git-flow)).
* Name a new branch according to its purpose in the format: [issue/bug/feature]/[your initials]/[issue number][description]
  * For example, to make a branch to fix Issue #0 about adding a README file, name the branch `issue/ABC/#0_add_readme_file`

### Code Styleguide

#### clang-format

OkapiLib uses the program [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to format code to meet a specific style (lightly modified LLVM style). Install the latest version and run `./run_clang-format.sh` to format all project files. Atom and VS Code, among other editors, have plugins that add support for running clang-format when you save a file. OkapiLib recommends these plugins so you stay on top of your formatting as you work.

#### cppcheck

OkapiLib uses the program [cppcheck](http://cppcheck.sourceforge.net/) for static analysis. Install the latest version and run `./run_cppcheck.sh` to check all project files.

### Other Points

* Every file must include the MPL2.0 header and attributions to applicable authors (first authors placed first).
* Use [Javadoc-style](https://www.tutorialspoint.com/java/java_documentation.htm) comments on constructors and methods.
* Name classes and methods using Camel case. Class names should start with a capital letter.
* Do not prefix member variables with `m_`; instead, prefix parameters with `i`.
* Place protected and private class members at the bottom of the class declaration.
* Do not `using namespace std` or any other namespace as this pollutes the user's namespace.
* No raw pointers. Use references (preferably `const` references) instead. If you need to pass an abstract class and can't use a reference, use a smart pointer.
* Use in-class member initializers and default constructors for no-arg constructors where possible.
* Use default arguments instead of method overloading.
* Everything that can be `const` should be.
* Don't optimize for no reason or prematurely optimize.
