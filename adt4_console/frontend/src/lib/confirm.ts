import type {JobConfirm} from "../types";

/** Whether the "yes" button may be pressed for `confirm` given the typed text. */
export function canAnswerYes(confirm: JobConfirm | null | undefined, typed: string): boolean {
  if (!confirm) return false;
  const req = confirm.required_text;
  if (req === null || req === undefined || req === "") return true;
  return typed === req;
}

/** Body for POST /api/jobs/{id}/confirm. */
export function confirmBody(confirm: JobConfirm, answer: boolean, typed: string): {answer: boolean; text?: string} {
  if (!answer) return {answer: false};
  return confirm.required_text ? {answer: true, text: typed} : {answer: true};
}
